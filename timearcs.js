
// Constants & SVGs
var margin = {top: 20, right: 20, bottom: 30, left: 140};
var width = document.body.clientWidth - margin.left - margin.right;
var height = document.body.clientHeight - margin.top - margin.bottom;

var svg = d3.select(".timearcs-panel").append("svg")
    .attr("width", width + margin.left + margin.right)
    .attr("height", height + margin.top + margin.bottom)
  .append("g")
    .attr("transform", "translate(" + margin.left + "," + margin.top + ")");

var tooltip = d3.select('#tooltip');

// Data file
var CSV_PATH = 'umdhusky-data_collection-syscall-auto-gps-no-cam-processed.csv';

// Accessors/Columns
var COL_EPOCH = 'Epoch Time';
var COL_NODE = 'Node';
var COL_SYSCALL = 'System Call';

// Scales
var xScale = d3.time.scale().range([0, Math.max(600, width - 260)]);
var yScale = d3.scale.ordinal(); // band positions per node
var arcWidthScale = d3.scale.linear().range([1, 2]);
var dotSizeScale = d3.scale.sqrt().range([2, 30]);
var colorScale = d3.scale.category20(); // Will be replaced with loaded colors

// Layout
var nodePadding = 20;
var rowHeight = 49;
var plotYOffset = 28; // push graph below the top time axis
var NUM_BINS = 120;
var activeSyscalls = new Set(); // Set of active system calls (empty = show all)
var callStatusFilter = 'all'; // 'all' | 'unfinished' | 'resumed' | 'normal'

// Globals for resize
var gState = { nodes: null, links: null, timeDomain: null, globalMaxBinTotal: 1, colors: null };
var redrawArcs = null; // Will be set by draw function

// Bundling visualization globals
var bundlingSvg = null;
var bundlingG = null;
var bundlingData = null;
var bundlingNodes = [];
var bundlingSyscalls = [];
var bundlingEdges = [];

// Time tick formatting (24h) that adapts to resolution
var multiTimeFormat = d3.time.format.multi([
  [".%L", function(d) { return d.getMilliseconds(); }], // Only for sub-second if needed
  ["%H:%M:%S", function() { return true; }] // Always show HH:MM:SS
]);

function setAxisTicks(axis, domain) {
  var spanMs = domain[1].getTime() - domain[0].getTime();
  // Dynamically adjust tick intervals
  if (spanMs <= 60 * 1000) { // Up to 1 minute, show every 5 seconds
    axis.ticks(d3.time.seconds, 5);
  } else if (spanMs <= 5 * 60 * 1000) { // Up to 5 minutes, show every 30 seconds
    axis.ticks(d3.time.seconds, 30);
  } else if (spanMs <= 30 * 60 * 1000) { // Up to 30 minutes, show every 5 minutes
    axis.ticks(d3.time.minutes, 5);
  } else {
    axis.ticks(d3.time.hours, 1); // Over 30 minutes, show every hour
  }
}

// Derived graph
// nodes: each unique Node plus central_os
// links: each row becomes a link from Node -> central_os at time t with syscall type

function parseRow(d) {
  var t = +d[COL_EPOCH]; // float seconds since epoch
  // guard: D3 v3 time scale expects Date
  return {
    epoch: t,
    date: new Date(t * 1000),
    node: d[COL_NODE],
    syscall: d[COL_SYSCALL],
    unfinished: +d['Unfinished Call'] || 0,
    resumed: +d['Resumed Call'] || 0
  };
}

function byKey(obj, key, init) {
  if (!obj.hasOwnProperty(key)) obj[key] = init;
  return obj[key];
}

function buildGraph(rows) {
  var nodeKeyToIndex = {};
  var nodes = [];
  var links = [];

  // Add central node
  nodeKeyToIndex['central_os'] = nodes.length;
  nodes.push({ id: 'central_os', label: 'central_os' });

  rows.forEach(function(r) {
    if (!r.node) return;
    if (!nodeKeyToIndex.hasOwnProperty(r.node)) {
      nodeKeyToIndex[r.node] = nodes.length;
      nodes.push({ id: r.node, label: r.node });
    }
    links.push({
      source: nodeKeyToIndex[r.node],
      target: nodeKeyToIndex['central_os'],
      date: r.date,
      epoch: r.epoch,
      syscall: r.syscall,
      unfinished: r.unfinished || 0,
      resumed: r.resumed || 0
    });
  });

  return { nodes: nodes, links: links, nodeKeyToIndex: nodeKeyToIndex };
}

function layoutNodes(nodes, links) {
  // Order nodes: all non-central alphabetically, central at center row
  var others = nodes.filter(function(n) { return n.id !== 'central_os'; })
                   .sort(function(a, b) { return d3.ascending(a.label, b.label); });
  var ordered = others.slice();
  ordered.splice(Math.floor(ordered.length/2), 0, nodes[0]); // insert central_os roughly in middle

  var totalHeight = Math.max(height, ordered.length * rowHeight + nodePadding*2);
  
  // Calculate spacing manually to respect rowHeight
  var startY = nodePadding;
  ordered.forEach(function(n, i) { 
    n.y = startY + i * rowHeight; 
  });
  
  // Set up yScale for reference but don't use it for positioning
  yScale.domain(ordered.map(function(n){return n.id;}))
        .range(ordered.map(function(n){return n.y;}));
  return { ordered: ordered, totalHeight: totalHeight };
}

function draw(nodes, links, timeDomain) {
  xScale.domain(timeDomain);

  d3.select('svg')
    .attr('height', height + margin.top + margin.bottom);

  // Axes
  var xAxis = d3.svg.axis().scale(xScale).orient('top').tickFormat(d3.time.format("%H:%M:%S"));
  setAxisTicks(xAxis, timeDomain);

  svg.append('g')
    .attr('class', 'x axis')
    .call(xAxis)
    .style('fill', gState.colors.ui.axis)
    .style('stroke', gState.colors.ui.axis);

  // Node labels and guide lines (created first for z-order)
  var nodeG = svg.append('g').attr('class', 'nodes');
  nodeG.selectAll('text.node-label')
    .data(nodes)
    .enter().append('text')
      .attr('class', 'node-label')
      .attr('x', -10)
      .attr('y', function(d){ return d.y + plotYOffset; })
      .attr('dy', '0.32em')
      .style('text-anchor', 'end')
      .style('font-size', '11px')
      .style('fill', gState.colors.ui.text)
      .text(function(d){ return d.label; })
      .on('mouseover', function(d){
        // Keep arcs hidden; only emphasize dots from this node
        arcG.selectAll('path.bin-arc').style('stroke-opacity', 0);
        // Fade all dots except those from this node
        arcG.selectAll('circle.arc-dot').style('opacity', function(arc) {
          return arc.nodeId === d.id ? 0.95 : 0.08;
        });
        // Include target-end dots
        arcG.selectAll('circle.arc-dot-target').style('opacity', function(arc) {
          return arc.nodeId === d.id ? 0.95 : 0.08;
        });
      })
      .on('mouseout', function(){
        // Keep arcs hidden by default
        arcG.selectAll('path.bin-arc').style('stroke-opacity', 0);
        arcG.selectAll('circle.arc-dot').style('opacity', 0.9);
        arcG.selectAll('circle.arc-dot-target').style('opacity', 0.9);
      });

  // Horizontal guideline lines removed

  // Zoom overlay
  var zoom = d3.behavior.zoom()
    .x(xScale)
    .scaleExtent([1, 100000])
    .on('zoom', onZoomed);

  var zoomPane = svg.append('rect')
    .attr('class', 'zoom-pane')
    .attr('x', 0)
    .attr('y', plotYOffset - 24)
    .attr('width', xScale.range()[1])
    .attr('height', Math.max(1000, height))
    .style('fill', 'transparent')
    .style('cursor', 'default')
    .style('pointer-events', 'none')
    .call(zoom);

  // Enable zoom only when user holds Shift
  var zoomKeyActive = false;
  function updateZoomKeyState(active) {
    zoomKeyActive = active;
    zoomPane.style('pointer-events', active ? 'all' : 'none')
            .style('cursor', active ? 'zoom-in' : 'default');
  }
  d3.select(window)
    .on('keydown.zoomtoggle', function() {
      var e = d3.event; if (!e) return;
      if (e.shiftKey) updateZoomKeyState(true);
    })
    .on('keyup.zoomtoggle', function() {
      updateZoomKeyState(false);
    })
    .on('blur.zoomtoggle', function(){ updateZoomKeyState(false); });

  // Prepare arc generator: arcs between (x(t), y(node)) and (x(t), y(central))
  var centralY = nodes.filter(function(n){return n.id==='central_os';})[0].y;

  // group links by time bucket for width aggregation (optional)
  // Here we simply use one path per link

  var arcG = svg.append('g').attr('class', 'arcs');

  redrawArcs = function() {
    var extent = xScale.domain();
    var spanMs = extent[1].getTime() - extent[0].getTime();
    var useBins = true; // always keep binning regardless of zoom level

    arcG.selectAll('*').remove();

    var nodeMinMaxX = {}; // To store min/max x for each node, moved outside if (useBins)

    if (useBins) {
      // Calculate time range in seconds and round up to nearest half or full minute
      var t0 = extent[0].getTime();
      var t1 = extent[1].getTime();
      
      // Calculate time range in seconds
      var timeRangeSeconds = (t1 - t0) / 1000;
      
      // Round up to the nearest half or full minute
      var minutes = Math.ceil(timeRangeSeconds / 30) * 0.5; // Round up to nearest 0.5 minute
      if (minutes < 1) minutes = 1; // Minimum 1 minute
      
      // Calculate number of bins: 120 bins per minute
      var totalBins = Math.ceil(minutes * 120);
      
      // Calculate step size in milliseconds
      var step = (t1 - t0) / totalBins;
      if (step <= 0) step = 1;

      var bins = d3.range(totalBins).map(function(i){
        return { i: i, t0: new Date(t0 + i*step), t1: new Date(t0 + (i+1)*step), countsByNode: {}, unfinishedByNodeByType: {}, resumedByNodeByType: {} };
      });

      // Filter links by call status first, then bin the filtered data
      var filteredLinks = links;
      if (callStatusFilter !== 'all') {
        filteredLinks = links.filter(function(l) {
          var hasUnfinished = l.unfinished === 1;
          var hasResumed = l.resumed === 1;
          
          switch (callStatusFilter) {
            case 'unfinished':
              return hasUnfinished;
            case 'resumed':
              return hasResumed;
            case 'normal':
              return !hasUnfinished && !hasResumed;
            default:
              return true;
          }
        });
      }

      filteredLinks.forEach(function(l){
        var tt = l.date.getTime();
        if (tt < t0 || tt > t1) return;
        var bi = Math.min(totalBins - 1, Math.max(0, Math.floor((tt - t0) / step)));
        var bin = bins[bi];
        var nodeId = nodes[l.source].id; // original nodes array here
        if (!bin.countsByNode[nodeId]) bin.countsByNode[nodeId] = { total: 0, byType: {} };
        bin.countsByNode[nodeId].total += 1;
        var ty = l.syscall;
        bin.countsByNode[nodeId].byType[ty] = (bin.countsByNode[nodeId].byType[ty] || 0) + 1;
        // accumulate unfinished/resumed per bin per node per system call type
        if (l.unfinished === 1) {
          if (!bin.unfinishedByNodeByType[nodeId]) bin.unfinishedByNodeByType[nodeId] = {};
          bin.unfinishedByNodeByType[nodeId][ty] = (bin.unfinishedByNodeByType[nodeId][ty] || 0) + 1;
        }
        if (l.resumed === 1) {
          if (!bin.resumedByNodeByType[nodeId]) bin.resumedByNodeByType[nodeId] = {};
          bin.resumedByNodeByType[nodeId][ty] = (bin.resumedByNodeByType[nodeId][ty] || 0) + 1;
        }
      });

      // Build per-node per-system-call entries from bins
      var entries = [];

      bins.forEach(function(b){
        for (var nid in b.countsByNode) {
          var n = b.countsByNode[nid];
          // Create separate entry for each system call type
          for (var syscallType in n.byType) {
            var count = n.byType[syscallType];
            if (count > 0) { // Only create entries for system calls that actually occurred
              entries.push({ 
                nodeId: nid, 
                bin: b, 
                maxType: syscallType, 
                total: count 
              });
            }
          }

          // Update min/max x for node (only need to do this once per node per bin)
          var binMidX = xScale(new Date((b.t0.getTime() + b.t1.getTime())/2));
          if (!nodeMinMaxX[nid]) {
            nodeMinMaxX[nid] = { minX: binMidX, maxX: binMidX };
          } else {
            nodeMinMaxX[nid].minX = Math.min(nodeMinMaxX[nid].minX, binMidX);
            nodeMinMaxX[nid].maxX = Math.max(nodeMinMaxX[nid].maxX, binMidX);
          }
        }
      });

      // Filter entries by active system calls (if any are selected)
      if (activeSyscalls.size > 0) {
        entries = entries.filter(function(entry) {
          return activeSyscalls.has(entry.maxType);
        });
      }
      
      // Note: Call status filtering is now done before binning, so no need to filter entries here
      
      // Sort entries by bin count (largest first) so smaller arcs are drawn on top
      entries.sort(function(a, b) { return b.total - a.total; });

      // Set linear domain for count-based thickness using max from filtered data
      var maxFilteredCount = 1;
      entries.forEach(function(entry) {
        if (entry.total > maxFilteredCount) maxFilteredCount = entry.total;
      });
      arcWidthScale.domain([1, Math.max(1, maxFilteredCount)]);
      // Use global max (computed at full extent) for dot sizing for consistency
      dotSizeScale.domain([1, Math.max(1, gState.globalMaxBinTotal)]);

      // Build status-specific entries for target dots (unfinished/resumed/normal)
      var statusEntries = [];
      bins.forEach(function(b){
        for (var nid in b.countsByNode) {
          var n = b.countsByNode[nid];
          for (var syscallType in n.byType) {
            var total = n.byType[syscallType] || 0;
            if (total <= 0) continue;
            var un = (b.unfinishedByNodeByType[nid] && b.unfinishedByNodeByType[nid][syscallType]) || 0;
            var re = (b.resumedByNodeByType[nid] && b.resumedByNodeByType[nid][syscallType]) || 0;
            var no = Math.max(0, total - un - re);
            if (un > 0) statusEntries.push({ nodeId: nid, bin: b, maxType: syscallType, total: un, status: 'unfinished' });
            if (re > 0) statusEntries.push({ nodeId: nid, bin: b, maxType: syscallType, total: re, status: 'resumed' });
            if (no > 0) statusEntries.push({ nodeId: nid, bin: b, maxType: syscallType, total: no, status: 'normal' });
          }
        }
      });
      // Apply active syscall filtering to statusEntries as well
      if (activeSyscalls.size > 0) {
        statusEntries = statusEntries.filter(function(entry){ return activeSyscalls.has(entry.maxType); });
      }
      // Sort status entries by size (largest first)
      statusEntries.sort(function(a, b){ return b.total - a.total; });

      arcG.selectAll('path.bin-arc')
        .data(entries)
        .enter().append('path')
          .attr('class', 'bin-arc')
          .style('fill', 'none')
          .style('stroke', function(d){
            // If filtering by status, color by status from colors.json
            if (callStatusFilter === 'unfinished') return gState.colors.statusColors.unfinished;
            if (callStatusFilter === 'resumed') return gState.colors.statusColors.resumed;
            if (callStatusFilter === 'normal') return gState.colors.statusColors.normal;
            return colorScale(d.maxType);
          })
          .style('stroke-opacity', 0) // hidden by default
          .style('pointer-events', 'none')
          .style('stroke-width', function(d){ return arcWidthScale(d.total); })
          .style('stroke-dasharray', function(d){
            // Since we're now filtering before binning, the dash pattern should reflect the filter type
            switch (callStatusFilter) {
              case 'unfinished':
                return '10,5'; // Unfinished: long dash
              case 'resumed':
                return '2,2'; // Resumed: short dash
              case 'normal':
                return 'none'; // Normal: solid line
              default:
                return 'none'; // Show all: solid line
            }
          })
          .attr('d', function(d){
            var idx = nodes.map(function(n){return n.id;}).indexOf(d.nodeId);
            var y1 = nodes[idx].y + plotYOffset;
            var centralIdx = nodes.map(function(n){return n.id;}).indexOf('central_os');
            var y2 = nodes[centralIdx].y + plotYOffset;
            var x = xScale(new Date((d.bin.t0.getTime() + d.bin.t1.getTime())/2));
            var dy = Math.abs(y2 - y1);
            var r = Math.max(6, dy/2);
            var yStart = Math.min(y1, y2), yEnd = Math.max(y1, y2);
            return 'M' + x + ',' + yStart + 'A' + r + ',' + r + ' 0 0,1 ' + x + ',' + yEnd;
          })
          .on('mouseover', function(d){
            tooltip.style('display','block').text(d.nodeId + ' — ' + d.maxType + ' (' + d.total + ')');
            // Dim other arcs
            arcG.selectAll('path.bin-arc')
              .style('stroke-opacity', function(x){ return x === d ? 0.95 : 0.12; });
            // Move the two labels (node and central_os) near the arc midpoint
            var midX = xScale(new Date((d.bin.t0.getTime() + d.bin.t1.getTime())/2));
            var hoveredNodeIdx = nodes.map(function(n){return n.id;}).indexOf(d.nodeId);
            var centralOSIdx = nodes.map(function(n){return n.id;}).indexOf('central_os');
            var hoveredNodeY = nodes[hoveredNodeIdx].y + plotYOffset;
            var centralOSY = nodes[centralOSIdx].y + plotYOffset;
            
            svg.selectAll('text.node-label')
              .filter(function(n){ return n.id === d.nodeId || n.id === 'central_os'; })
              .transition().duration(120)
              .attr('x', function(n){ return midX + (n.id === 'central_os' ? -12 : 12); })
              .attr('y', function(n){ return n.id === 'central_os' ? centralOSY + 5 : hoveredNodeY - 5; })
              .style('text-anchor', function(n){ return n.id === 'central_os' ? 'end' : 'start'; });
          })
          .on('mousemove', function(){ var e=d3.event; tooltip.style('left',(e.clientX+10)+'px').style('top',(e.clientY+10)+'px'); })
          .on('mouseout', function(d){
            tooltip.style('display','none');
            // Restore arc opacities
            arcG.selectAll('path.bin-arc').style('stroke-opacity', 0);
            // Restore dot opacities
            arcG.selectAll('circle.arc-dot').style('opacity', 0.9);
            arcG.selectAll('circle.arc-dot-target').style('opacity', 0.9);
            // Restore moved labels to left gutter
            svg.selectAll('text.node-label')
              .filter(function(n){ return n.id === 'central_os' || n.id === d.nodeId; })
              .transition().duration(120)
              .attr('x', -10)
              .attr('y', function(n){
                return n.y + plotYOffset; // Directly use n.y, which is the original layout y
              })
              .style('text-anchor', 'end');
          });

      // Directional dots at source ends for each binned arc entry
      arcG.selectAll('circle.arc-dot')
        .data(entries)
        .enter().append('circle')
          .attr('class', 'arc-dot')
          .attr('cx', function(d){
            return xScale(new Date((d.bin.t0.getTime() + d.bin.t1.getTime())/2));
          })
          .attr('cy', function(d){
            var idx = nodes.map(function(n){return n.id;}).indexOf(d.nodeId);
            var y1 = nodes[idx].y + plotYOffset;
            return y1; // source node end
          })
          .attr('r', function(d){ return dotSizeScale(d.total); })
          .style('fill', function(d){ return colorScale(d.maxType); })
          .style('opacity', 0.9)
          .on('mouseover', function(d){
            tooltip.style('display','block').text(d.nodeId + ' — ' + d.maxType + ' (' + d.total + ')');
            // Highlight matching arcs and dots for this specific system call in the same node/bin
            arcG.selectAll('path.bin-arc')
              .style('stroke-opacity', function(x){
                return (x.nodeId === d.nodeId && x.bin.i === d.bin.i && x.maxType === d.maxType) ? 0.95 : 0;
              });
            arcG.selectAll('circle.arc-dot')
              .style('opacity', function(x){
                return (x.nodeId === d.nodeId && x.bin.i === d.bin.i && x.maxType === d.maxType) ? 0.95 : 0.12;
              });
            arcG.selectAll('circle.arc-dot-target')
              .style('opacity', function(x){
                return (x.nodeId === d.nodeId && x.bin.i === d.bin.i && x.maxType === d.maxType) ? 0.95 : 0.12;
              });
          })
          .on('mousemove', function(){ var e=d3.event; tooltip.style('left',(e.clientX+10)+'px').style('top',(e.clientY+10)+'px'); })
          .on('mouseout', function(){
            tooltip.style('display','none');
            // Hide arcs again when not hovering a dot
            arcG.selectAll('path.bin-arc').style('stroke-opacity', 0);
            arcG.selectAll('circle.arc-dot').style('opacity', 0.9);
            arcG.selectAll('circle.arc-dot-target').style('opacity', 0.9);
          });

      // Dots at the central_os end, colored by status scheme using statusEntries
      arcG.selectAll('circle.arc-dot-target')
        .data(statusEntries)
        .enter().append('circle')
          .attr('class', 'arc-dot-target')
          .attr('cx', function(d){
            return xScale(new Date((d.bin.t0.getTime() + d.bin.t1.getTime())/2));
          })
          .attr('cy', function(d){
            var centralIdx = nodes.map(function(n){return n.id;}).indexOf('central_os');
            return nodes[centralIdx].y + plotYOffset;
          })
          .attr('r', function(d){ return dotSizeScale(d.total); })
          .style('fill', function(d){
            if (d.status === 'unfinished') return gState.colors.statusColors.unfinished;
            if (d.status === 'resumed') return gState.colors.statusColors.resumed;
            return gState.colors.statusColors.normal; // normal
          })
          .style('opacity', 0.9)
          .on('mouseover', function(d){
            tooltip.style('display','block').text('central_os — ' + d.maxType + ' (' + d.total + ')');
            arcG.selectAll('path.bin-arc')
              .style('stroke-opacity', function(x){
                return (x.nodeId === d.nodeId && x.bin.i === d.bin.i && x.maxType === d.maxType) ? 0.95 : 0;
              });
            arcG.selectAll('circle.arc-dot')
              .style('opacity', function(x){
                return (x.nodeId === d.nodeId && x.bin.i === d.bin.i && x.maxType === d.maxType) ? 0.95 : 0.12;
              });
            arcG.selectAll('circle.arc-dot-target')
              .style('opacity', function(x){
                return (x.nodeId === d.nodeId && x.bin.i === d.bin.i && x.maxType === d.maxType) ? 0.95 : 0.12;
              });
          })
          .on('mousemove', function(){ var e=d3.event; tooltip.style('left',(e.clientX+10)+'px').style('top',(e.clientY+10)+'px'); })
          .on('mouseout', function(){
            tooltip.style('display','none');
            arcG.selectAll('path.bin-arc').style('stroke-opacity', 0);
            arcG.selectAll('circle.arc-dot').style('opacity', 0.9);
            arcG.selectAll('circle.arc-dot-target').style('opacity', 0.9);
          });


    } else {
      // Raw arcs in detailed view - This block is currently not used due to useBins = true
      // However, nodeMinMaxX calculation would be needed for raw arcs too if this block were active
      // For now, setting x1/x2 to the same value if no bins/arcs found handles this implicitly
      arcG.selectAll('path.arc')
        .data(links.filter(function(l){ return l.date >= extent[0] && l.date <= extent[1]; }))
        .enter().append('path')
          .attr('class', 'arc')
          .style('fill', 'none')
          .style('stroke', function(d){ return colorScale(d.syscall); })
          .style('stroke-width', function(d){ return arcWidthScale(1); })
          .style('stroke-opacity', 0.9)
          .attr('d', function(d){
            var x = xScale(d.date);
            var y1 = nodes[d.source].y + plotYOffset;
            var y2 = nodes[d.target].y + plotYOffset;
            var dy = Math.abs(y2 - y1);
            var r = Math.max(6, dy/2);
            var yStart = Math.min(y1, y2), yEnd = Math.max(y1, y2);
            return 'M' + x + ',' + yStart + 'A' + r + ',' + r + ' 0 0,1 ' + x + ',' + yEnd;
          })
          .on('mouseover', function(d){
            tooltip.style('display', 'block')
                   .text(d.syscall + ' @ ' + d.epoch + ' — ' + nodes[d.source].label + ' → central_os');
          })
          .on('mousemove', function(){ var e=d3.event; tooltip.style('left',(e.clientX+10)+'px').style('top',(e.clientY+10)+'px'); })
          .on('mouseout', function(){ tooltip.style('display', 'none'); });
    }

    // Guidelines removed - no longer updating horizontal lines

    // Update axis
    var xAxis2 = d3.svg.axis().scale(xScale).orient('top').tickFormat(d3.time.format("%H:%M:%S"));
    setAxisTicks(xAxis2, xScale.domain());
    svg.select('g.x.axis').call(xAxis2);
  }

  function onZoomed() {
    redrawArcs();
    // The guidelines are updated within redrawArcs now, so this might not be needed for x2
    // svg.selectAll('line.node-guideline').attr('x2', xScale.range()[1]);
  }


  // initial render
  redrawArcs();

  // Keyboard and wheel panning when zoomed in
  function isZoomedIn() {
    var dom = xScale.domain();
    var spanMs = dom[1].getTime() - dom[0].getTime();
    var fullMs = timeDomain[1].getTime() - timeDomain[0].getTime();
    return spanMs < fullMs; // true when zoomed in
  }

  function panByPixels(px) {
    if (px === 0) return;
    var dom = xScale.domain();
    var spanMs = dom[1].getTime() - dom[0].getTime();
    var rng = xScale.range();
    var widthPx = rng[1] - rng[0];
    if (widthPx <= 0) return;
    var deltaMs = px / widthPx * spanMs;
    xScale.domain([ new Date(dom[0].getTime() + deltaMs), new Date(dom[1].getTime() + deltaMs) ]);
    redrawArcs();
  }

  d3.select(window)
    .on('keydown.pan', function() {
      var e = d3.event; if (!e) return;
      if (!isZoomedIn()) return;
      if (e.key === 'ArrowLeft' || e.key === 'a' || e.key === 'A') {
        e.preventDefault();
        panByPixels(-0.1 * (xScale.range()[1] - xScale.range()[0]));
      } else if (e.key === 'ArrowRight' || e.key === 'd' || e.key === 'D') {
        e.preventDefault();
        panByPixels(0.1 * (xScale.range()[1] - xScale.range()[0]));
      }
    })
    .on('wheel.hpan', function() {
      var e = d3.event; if (!e) return;
      if (!isZoomedIn()) return;
      // Use horizontal wheel/trackpad motion for pan when not holding Shift (reserved for zoom)
      if (!e.shiftKey && Math.abs(e.deltaX) > Math.abs(e.deltaY)) {
        try { e.preventDefault(); } catch(_) {}
        panByPixels(e.deltaX);
      }
    });

  // Zoom-to-fit: reset domain to full dataset extent
  function zoomToFit() {
    xScale.domain(timeDomain);
    redrawArcs();
  }

  // Add a small fixed-position button
  if (d3.select('#zoomFitBtn').empty()) {
    d3.select('body').append('button')
      .attr('id', 'zoomFitBtn')
      .text('Fit')
      .style('position', 'fixed')
      .style('bottom', '16px')
      .style('right', '16px')
      .style('padding', '6px 10px')
      .style('border', '1px solid #ccc')
      .style('border-radius', '4px')
      .style('background', '#fff')
      .style('cursor', 'pointer')
      .on('click', function(){ zoomToFit(); });
  }

  // Keyboard shortcut: press 'f' to fit
  d3.select(window).on('keydown.fit', function(){
    var e = d3.event; if (!e) return;
    if (e.key === 'f' || e.key === 'F') { e.preventDefault(); zoomToFit(); }
  });
}

function buildLegend(syscalls) {
  var legend = d3.select('#legend');
  
  // Apply legend styling from loaded colors
  legend.style('background', 'rgba(0,0,0,0.8)')
         .style('color', gState.colors.ui.text)
         .style('border-color', gState.colors.ui.button.border);
  
  var items = legend.selectAll('div.legend-item')
    .data(syscalls)
    .enter().append('div')
      .attr('class', 'legend-item')
      .style('cursor', 'pointer')
      .style('opacity', function(d) { 
        return activeSyscalls.size === 0 || activeSyscalls.has(d) ? 1.0 : 0.3; 
      })
      .on('click', function(d) {
        // Single click: toggle hide/show for this item
        if (activeSyscalls.has(d)) {
          activeSyscalls.delete(d);
        } else {
          activeSyscalls.add(d);
        }
        
        // Update legend opacity
        legend.selectAll('div.legend-item')
          .style('opacity', function(syscall) { 
            return activeSyscalls.size === 0 || activeSyscalls.has(syscall) ? 1.0 : 0.3; 
          });
        
        // Redraw arcs with new filter
        redrawArcs();
        renderBundling();
      })
      .on('dblclick', function(d) {
        // Double click: isolate this item (hide all others)
        activeSyscalls.clear();
        activeSyscalls.add(d);
        
        // Update legend opacity
        legend.selectAll('div.legend-item')
          .style('opacity', function(syscall) { 
            return syscall === d ? 1.0 : 0.3; 
          });
        
        // Redraw arcs with new filter
        redrawArcs();
        renderBundling();
      });
      
  items.append('span')
    .attr('class', 'swatch')
    .style('background', function(d){ return colorScale(d); });
  items.append('span').text(function(d){ 
    var freq = gState.colors.systemCallFrequency[d] || 0;
    return d + ' (' + freq.toLocaleString() + ')';
  });
  
}


// Size legend for circle (dot) counts using global max
function buildSizeLegend() {
  // Ensure container exists
  var container = d3.select('#sizeLegend');
  if (container.empty()) {
    container = d3.select('body').append('div')
      .attr('id', 'sizeLegend')
      .style('position', 'fixed')
      .style('padding', '6px 8px')
      .style('background', 'transparent')
      .style('border', 'none')
      .style('border-radius', '4px')
      .style('color', '#ffffff')
       .style('font-size', '11px')
       .style('z-index', 'auto');
  }

  // Clear and (re)build SVG
  container.html('');
  // Compute dynamic layout so border tightly fits circles and title
  var maxCount = Math.max(1, gState.globalMaxBinTotal || 1);
  var minCount = 1;
  var midCount = Math.max(1, Math.round((maxCount + minCount) / 2));
  var values = [maxCount, midCount, minCount];

  var padding = 10;
  var titleY = 18;
  var gapBelowTitle = 20; // extra space so circles sit clearly below title
  var maxR = dotSizeScale(maxCount);
  var cx = padding + maxR;
  // Place the TOP of the largest circle gapBelowTitle below the title
  // Using baseY (bottom alignment) ensures smaller circles nest correctly
  var baseY = titleY + gapBelowTitle + 2 * maxR;

  var svgW = padding * 2 + maxR * 2;
  var svgH = padding + titleY + gapBelowTitle + maxR * 2 + padding + maxR; // extra breathing room

  var s = container.append('svg')
    .attr('width', svgW)
    .attr('height', svgH);

  // Border around legend (tight fit)
  s.append('rect')
    .attr('x', 0)
    .attr('y', 0)
    .attr('width', svgW)
    .attr('height', svgH)
    .attr('rx', 6)
    .attr('ry', 6)
    .style('fill', 'transparent')
    .style('stroke', '#ffffff')
    .style('stroke-width', 1);

  // Title
  s.append('text')
    .attr('x', padding)
    .attr('y', titleY)
    .style('font-size', '12px')
    .style('font-weight', '600')
    .style('fill', '#ffffff')
    .text('Count scale');


  // Use the same radius scale as dots
  values.forEach(function(v, i) {
    var r = dotSizeScale(v);
    var cy = baseY - r; // align bottoms

    // Circle
    s.append('circle')
      .attr('cx', cx)
      .attr('cy', cy)
      .attr('r', r)
      .style('fill', gState.colors && gState.colors.ui ? (gState.colors.ui.background || 'transparent') : 'transparent')
      .style('stroke', '#ffffff')
      .style('stroke-width', 1);

    // Label above each circle; offset to avoid overlap
    var yOffset = (i === 0) ? -8 : -4;
    s.append('text')
      .attr('x', cx)
      .attr('y', cy - r + yOffset)
      .style('fill', '#ffffff')
      .style('text-anchor', 'middle')
      .style('font-size', '10px')
      .text(v.toLocaleString());
  });
}


function onResize() {
  if (!gState.nodes || !gState.links) return;
  
  // Update time arcs view
  var outer = d3.select('.timearcs-panel svg');
  var newWidth = document.body.clientWidth - margin.left - margin.right;
  width = newWidth;
  xScale.range([0, Math.max(600, width - 260)]);
  outer.attr('width', width + margin.left + margin.right)
       .attr('height', document.body.clientHeight);

  // Update axis
  var xAxis = d3.svg.axis().scale(xScale).orient('top').tickFormat(d3.time.format("%H:%M:%S"));
  setAxisTicks(xAxis, gState.timeDomain);
  svg.select('g.x.axis').call(xAxis);

  // Update axis again based on possibly changed domain
  var xAxis2 = d3.svg.axis().scale(xScale).orient('top').tickFormat(d3.time.format("%H:%M:%S"));
  setAxisTicks(xAxis2, xScale.domain());
  svg.select('g.x.axis').call(xAxis2);

  // Update node label positions
  svg.selectAll('text.node-label')
    .attr('y', function(d){ return d.y + plotYOffset; });

  // Update bundling view (only if popup is visible)
  if (bundlingSvg && d3.select('#bundlingPopup').style('display') !== 'none') {
    var popupWidth = 600; // Fixed width
    var popupHeight = 480; // Fixed height minus even smaller header
    
    bundlingSvg.attr('width', popupWidth)
               .attr('height', popupHeight);
    
    // Update bundling transform
    bundlingG.attr('transform', 'translate(' + (popupWidth/2) + ',' + (popupHeight/2) + ')');
    
    // Re-render bundling
    renderBundling();
  }
    
  // Reposition buttons below legend after resize
  positionButtonsBelowLegend();

  // Reposition size legend relative to main legend
  var legend = document.getElementById('legend');
  var sizeLegend = document.getElementById('sizeLegend');
  if (legend && sizeLegend) {
    var rect = legend.getBoundingClientRect();
    sizeLegend.style.left = rect.left + 'px';
    sizeLegend.style.top = (rect.bottom + window.scrollY + 8) + 'px';
  }

  // Use a simple vertical stack: legend -> sizeLegend -> filter buttons -> bundling button
  var buttonContainer = document.getElementById('filterToggleBtns');
  var bundlingButton = document.getElementById('bundlingTriggerBtn');
  if (legend) {
    var y = legend.getBoundingClientRect().bottom + window.scrollY + 8;
    if (sizeLegend) {
      sizeLegend.style.left = legend.style.left || (legend.getBoundingClientRect().left + 'px');
      sizeLegend.style.top = y + 'px';
      // If sizeLegend uses fixed positioning, compute bottom using its height
      var slRect = sizeLegend.getBoundingClientRect();
      y = (slRect.top + slRect.height) + 8;
    }
    if (buttonContainer) {
      buttonContainer.style.position = 'fixed';
      buttonContainer.style.left = (legend.getBoundingClientRect().left + 'px');
      buttonContainer.style.top = y + 'px';
      y = buttonContainer.getBoundingClientRect().bottom + 8;
    }
    if (bundlingButton) {
      bundlingButton.style.position = 'fixed';
      bundlingButton.style.left = (legend.getBoundingClientRect().left + 'px');
      bundlingButton.style.top = y + 'px';
    }
  }
}

// Bundling visualization functions
function initBundling() {
  // Create bundling SVG in the popup body
  bundlingSvg = d3.select(".bundling-popup-body").append("svg")
    .attr("class", "bundling-svg")
    .attr("width", "100%")
    .attr("height", "100%");

  bundlingG = bundlingSvg.append('g')
    .attr('transform', 'translate(' + (document.body.clientWidth/2) + ',' + (document.body.clientHeight/2) + ')');
}

function processBundlingData(rows) {
  // Get unique nodes and system calls
  var nodeSet = d3.set();
  var syscallSet = d3.set();
  var connections = {};

  rows.forEach(function(d) {
    nodeSet.add(d.node);
    syscallSet.add(d.syscall);
    
    var key = d.node + '-' + d.syscall;
    if (!connections[key]) {
      connections[key] = {
        node: d.node,
        syscall: d.syscall,
        count: 0,
        unfinished: 0,
        resumed: 0
      };
    }
    var conn = connections[key];
    conn.count++;
    if (d.unfinished) conn.unfinished++;
    if (d.resumed) conn.resumed++;
  });

  // Create nodes array
  bundlingNodes = nodeSet.values().map(function(node) {
    return {
      id: node,
      label: node,
      type: 'node'
    };
  });

  // Create syscalls array
  bundlingSyscalls = syscallSet.values().map(function(syscall) {
    return {
      id: syscall,
      label: syscall,
      type: 'syscall',
      frequency: gState.colors.systemCallFrequency[syscall] || 0
    };
  }).sort(function(a, b) { return b.frequency - a.frequency; });

  // Create edges array
  bundlingEdges = Object.keys(connections).map(function(key) {
    return connections[key];
  });

  bundlingData = rows;
  
}

function renderBundling() {
  if (!bundlingG || !bundlingNodes.length) return;

  // Clear previous render
  bundlingG.selectAll('*').remove();

  // Get popup dimensions for radius calculation (fixed size)
  var popupWidth = 600;
  var popupHeight = 480;
  var radius = Math.min(popupWidth, popupHeight) * 0.35; // Bigger radius for larger bundling
  var nodeRadius = 8;
  var syscallRadius = 6;

  // Background first (behind everything) - draw immediately after clearing
  bundlingG.append('circle')
    .attr('cx', 0)
    .attr('cy', 0)
    .attr('r', radius + 20)
    .style('fill', 'rgba(255, 255, 255, 0.08)')
    .style('stroke', '#555')
    .style('stroke-width', 1);
  // Semicircle guides
  bundlingG.append('path')
    .attr('d', 'M ' + (-radius - 10) + ' 0 A ' + (radius + 10) + ' ' + (radius + 10) + ' 0 0 1 ' + (radius + 10) + ' 0')
    .style('fill', 'none')
    .style('stroke', 'rgba(255, 255, 255, 0.1)')
    .style('stroke-width', 1)
    .style('stroke-dasharray', '5,5');
  bundlingG.append('path')
    .attr('d', 'M ' + (-radius - 10) + ' 0 A ' + (radius + 10) + ' ' + (radius + 10) + ' 0 0 0 ' + (radius + 10) + ' 0')
    .style('fill', 'none')
    .style('stroke', 'rgba(255, 255, 255, 0.1)')
    .style('stroke-width', 1)
    .style('stroke-dasharray', '5,5');

  // Calculate positions for nodes (left half of circle)
  // Use wider range from 2π/3 to 4π/3 (120° to 240°) to spread them out but avoid ends
  var nodeStartAngle = 2 * Math.PI / 3;
  var nodeEndAngle = 4 * Math.PI / 3;
  bundlingNodes.forEach(function(node, i) {
    var angle = nodeStartAngle + (i * (nodeEndAngle - nodeStartAngle) / Math.max(1, bundlingNodes.length - 1));
    node.x = Math.cos(angle) * radius;
    node.y = Math.sin(angle) * radius;
  });

  // Calculate positions for system calls (right half of circle)
  // Use wider range from -π/3 to π/3 (-60° to 60°) to spread them out but avoid ends
  var syscallStartAngle = -Math.PI / 3;
  var syscallEndAngle = Math.PI / 3;
  bundlingSyscalls.forEach(function(syscall, i) {
    var angle = syscallStartAngle + (i * (syscallEndAngle - syscallStartAngle) / Math.max(1, bundlingSyscalls.length - 1));
    syscall.x = Math.cos(angle) * radius;
    syscall.y = Math.sin(angle) * radius;
  });

  // Filter edges based on current filters
  var filteredEdges = bundlingEdges.filter(function(edge) {
    // Filter by call status
    var statusMatch = true;
    if (callStatusFilter !== 'all') {
      var hasUnfinished = edge.unfinished > 0;
      var hasResumed = edge.resumed > 0;
      
      switch (callStatusFilter) {
        case 'unfinished':
          statusMatch = hasUnfinished;
          break;
        case 'resumed':
          statusMatch = hasResumed;
          break;
        case 'normal':
          statusMatch = !hasUnfinished && !hasResumed;
          break;
      }
    }

    // Filter by active system calls
    var syscallMatch = activeSyscalls.size === 0 || activeSyscalls.has(edge.syscall);

    return statusMatch && syscallMatch;
  });

  // Bin edges by node-syscall pairs and sum their counts
  var binnedEdges = {};
  filteredEdges.forEach(function(edge) {
    var key = edge.node + '-' + edge.syscall;
    if (!binnedEdges[key]) {
      binnedEdges[key] = {
        node: edge.node,
        syscall: edge.syscall,
        count: 0,
        unfinished: 0,
        resumed: 0
      };
    }
    binnedEdges[key].count += edge.count;
    binnedEdges[key].unfinished += edge.unfinished;
    binnedEdges[key].resumed += edge.resumed;
  });

  var finalEdges = Object.keys(binnedEdges).map(function(key) {
    return binnedEdges[key];
  });

  // Calculate max count for thickness scaling - show actual bin sizes
  var maxCount = d3.max(finalEdges, function(d) { return d.count; }) || 1;
  console.log('Bundling maxCount:', maxCount, 'finalEdges length:', finalEdges.length);
  
  // Create bundling-specific thickness scale to show bin sizes clearly
  var bundlingThicknessScale = d3.scale.linear()
    .domain([1, maxCount])
    .range([1, 12]); // Much wider range to show bin size differences

  // Draw edges
  var edgeGroup = bundlingG.append('g').attr('class', 'edges');
  
  var edgePaths = edgeGroup.selectAll('.edge')
    .data(finalEdges)
    .enter()
    .append('path')
    .attr('class', 'edge')
    .attr('d', function(d) {
      var sourceNode = null;
      for (var i = 0; i < bundlingNodes.length; i++) {
        if (bundlingNodes[i].id === d.node) {
          sourceNode = bundlingNodes[i];
          break;
        }
      }
      var targetSyscall = null;
      for (var j = 0; j < bundlingSyscalls.length; j++) {
        if (bundlingSyscalls[j].id === d.syscall) {
          targetSyscall = bundlingSyscalls[j];
          break;
        }
      }
      
      if (!sourceNode || !targetSyscall) return '';
      
      // Create curved path using quadratic bezier with control point towards center
      var midX = (sourceNode.x + targetSyscall.x) / 2;
      var midY = (sourceNode.y + targetSyscall.y) / 2;
      var controlX = midX * 0.3; // Pull curve towards center
      var controlY = midY * 0.3;
      
      return 'M ' + sourceNode.x + ' ' + sourceNode.y + ' Q ' + controlX + ' ' + controlY + ' ' + targetSyscall.x + ' ' + targetSyscall.y;
    })
    .style('stroke', function(d) { return colorScale(d.syscall); })
    .style('stroke-width', function(d) { 
      var width = bundlingThicknessScale(d.count);
      if (d.count > 100) console.log('High count edge:', d.node, d.syscall, d.count, 'width:', width);
      return width;
    })
    .style('stroke-opacity', 0.6)
    .style('fill', 'none')
    .on('mouseover', function(d) {
      tooltip.style('display','block').text(d.node + ' — ' + d.syscall + ' (' + d.count + ')');
      
      // Highlight this edge (opacity only, keep original thickness)
      d3.select(this).style('stroke-opacity', 1);
      
      // Dim other edges
      edgeGroup.selectAll('.edge').style('stroke-opacity', 0.1);
      d3.select(this).style('stroke-opacity', 1);

      // Highlight only the two endpoint nodes for this edge; dim others
      nodeGroup.selectAll('.node-circle')
        .style('opacity', function(n) { return n.id === d.node ? 1 : 0.1; });
      syscallGroup.selectAll('.syscall-circle')
        .style('opacity', function(s) { return s.id === d.syscall ? 1 : 0.1; });
      
      // Highlight corresponding arcs in time arcs visualization (only selected)
      svg.selectAll('path.bin-arc').style('stroke-opacity', function(arc) {
        return (arc.nodeId === d.node && arc.maxType === d.syscall) ? 1 : 0;
      });
    })
    .on('mousemove', function(){ var e=d3.event; tooltip.style('left',(e.clientX+10)+'px').style('top',(e.clientY+10)+'px'); })
    .on('mouseout', function() {
      tooltip.style('display','none');
      edgeGroup.selectAll('.edge').style('stroke-opacity', 0.6).style('stroke-width', function(d) { return bundlingThicknessScale(d.count); });
      // Restore node/syscall circles
      nodeGroup.selectAll('.node-circle').style('opacity', 1);
      syscallGroup.selectAll('.syscall-circle').style('opacity', 1);
      
      // Keep time arcs hidden by default
      svg.selectAll('path.bin-arc').style('stroke-opacity', 0);
    });

  // Draw nodes (left half)
  var nodeGroup = bundlingG.append('g').attr('class', 'nodes');
  
  nodeGroup.selectAll('.node-circle')
    .data(bundlingNodes)
    .enter()
    .append('circle')
    .attr('class', 'node-circle')
    .attr('cx', function(d) { return d.x; })
    .attr('cy', function(d) { return d.y; })
    .attr('r', nodeRadius)
    .style('fill', '#4a90e2')
    .style('stroke', '#ffffff')
    .style('stroke-width', 2)
    .on('mouseover', function(d) {
      tooltip.style('display','block').text('Node: ' + d.label);
      
      // Highlight connections from this node
      edgeGroup.selectAll('.edge')
        .style('stroke-opacity', function(edge) { return edge.node === d.id ? 1 : 0.1; });
      // Highlight only this node and connected syscalls; dim others
      nodeGroup.selectAll('.node-circle').style('opacity', function(n) { return n.id === d.id ? 1 : 0.1; });
      var connectedSyscalls = {};
      finalEdges.forEach(function(e){ if (e.node === d.id) connectedSyscalls[e.syscall] = true; });
      syscallGroup.selectAll('.syscall-circle')
        .style('opacity', function(s) { return connectedSyscalls[s.id] ? 1 : 0.1; });
      
      // Highlight corresponding arcs in time arcs visualization (only selected)
      svg.selectAll('path.bin-arc').style('stroke-opacity', function(arc) {
        return arc.nodeId === d.id ? 1 : 0;
      });
    })
    .on('mousemove', function(){ var e=d3.event; tooltip.style('left',(e.clientX+10)+'px').style('top',(e.clientY+10)+'px'); })
    .on('mouseout', function() {
      tooltip.style('display','none');
      edgeGroup.selectAll('.edge').style('stroke-opacity', 0.6);
      nodeGroup.selectAll('.node-circle').style('opacity', 1);
      syscallGroup.selectAll('.syscall-circle').style('opacity', 1);
      
      // Keep time arcs hidden by default
      svg.selectAll('path.bin-arc').style('stroke-opacity', 0);
    });

  // Draw node labels (outside circle)
  nodeGroup.selectAll('.node-text')
    .data(bundlingNodes)
    .enter()
    .append('text')
    .attr('class', 'node-text')
    .attr('x', function(d) { 
      // Position labels outside the circle
      var angle = Math.atan2(d.y, d.x);
      return Math.cos(angle) * (radius + 30);
    })
    .attr('y', function(d) { 
      // Position labels outside the circle
      var angle = Math.atan2(d.y, d.x);
      return Math.sin(angle) * (radius + 30);
    })
    .style('font-size', '10px')
    .style('text-anchor', 'end')
    .style('dominant-baseline', 'middle')
    .style('fill', '#ffffff')
    .text(function(d) { return d.label.length > 15 ? d.label.substring(0, 15) + '...' : d.label; });

  // Draw system calls (right half)
  var syscallGroup = bundlingG.append('g').attr('class', 'syscalls');
  
  syscallGroup.selectAll('.syscall-circle')
    .data(bundlingSyscalls)
    .enter()
    .append('circle')
    .attr('class', 'syscall-circle')
    .attr('cx', function(d) { return d.x; })
    .attr('cy', function(d) { return d.y; })
    .attr('r', syscallRadius)
    .style('fill', function(d) { return colorScale(d.id); })
    .style('stroke', '#ffffff')
    .style('stroke-width', 1)
    .on('mouseover', function(d) {
      tooltip.style('display','block').text('System Call: ' + d.label + ' (Frequency: ' + d.frequency.toLocaleString() + ')');
      
      // Highlight connections to this syscall
      edgeGroup.selectAll('.edge')
        .style('stroke-opacity', function(edge) { return edge.syscall === d.id ? 1 : 0.1; });
      // Highlight only this syscall and connected nodes; dim others
      syscallGroup.selectAll('.syscall-circle').style('opacity', function(s) { return s.id === d.id ? 1 : 0.1; });
      var connectedNodes = {};
      finalEdges.forEach(function(e){ if (e.syscall === d.id) connectedNodes[e.node] = true; });
      nodeGroup.selectAll('.node-circle')
        .style('opacity', function(n) { return connectedNodes[n.id] ? 1 : 0.1; });
      
      // Highlight corresponding arcs in time arcs visualization (only selected)
      svg.selectAll('path.bin-arc').style('stroke-opacity', function(arc) {
        return arc.maxType === d.id ? 1 : 0;
      });
    })
    .on('mousemove', function(){ var e=d3.event; tooltip.style('left',(e.clientX+10)+'px').style('top',(e.clientY+10)+'px'); })
    .on('mouseout', function() {
      tooltip.style('display','none');
      edgeGroup.selectAll('.edge').style('stroke-opacity', 0.6);
      nodeGroup.selectAll('.node-circle').style('opacity', 1);
      syscallGroup.selectAll('.syscall-circle').style('opacity', 1);
      
      // Keep time arcs hidden by default
      svg.selectAll('path.bin-arc').style('stroke-opacity', 0);
    });

  // Draw syscall labels (outside circle)
  syscallGroup.selectAll('.syscall-text')
    .data(bundlingSyscalls)
    .enter()
    .append('text')
    .attr('class', 'syscall-text')
    .attr('x', function(d) { 
      // Position labels outside the circle
      var angle = Math.atan2(d.y, d.x);
      return Math.cos(angle) * (radius + 25);
    })
    .attr('y', function(d) { 
      // Position labels outside the circle
      var angle = Math.atan2(d.y, d.x);
      return Math.sin(angle) * (radius + 25);
    })
    .style('font-size', '9px')
    .style('text-anchor', 'start')
    .style('dominant-baseline', 'middle')
    .style('fill', '#ffffff')
    .text(function(d) { return d.label; });

  // (Background was moved to bgG at the top of renderBundling)
}



function loadDataset(csvPath) {
  // Colors are loaded once in init(); reuse them here.
  var colors = gState.colors;

  // --- Tear down any previous render so datasets don't stack/overlap ---
  // Reset filter state to "Show All" on dataset switch
  activeSyscalls.clear();
  callStatusFilter = 'all';
  d3.selectAll('#filterToggleBtns button').classed('active', false);
  d3.select('#showAllFilterBtn').classed('active', true);

  // Clear the main SVG group (axes, node labels, arcs) and the legend
  svg.selectAll('*').remove();
  d3.select('#legend').selectAll('*').remove();
  redrawArcs = null;

  // Reset bundling render + module-level bundling state
  d3.select('.bundling-popup-body').selectAll('svg').remove();
  bundlingSvg = null;
  bundlingG = null;
  bundlingData = null;
  bundlingNodes = [];
  bundlingSyscalls = [];
  bundlingEdges = [];

  // Reset shared graph state
  gState.nodes = null;
  gState.links = null;
  gState.timeDomain = null;
  gState.globalMaxBinTotal = 1;

  // Load and process data for the chosen dataset
  d3.csv(csvPath, function(err, raw) {
    if (err) { console.error(err); return; }

      // Map
      var rows = raw.map(parseRow).filter(function(r){ return isFinite(r.epoch) && r.node && r.syscall; });
      if (rows.length === 0) { return; }

      // Time domain: from min to max
      var minDate = d3.min(rows, function(r){ return r.date; });
      var maxDate = d3.max(rows, function(r){ return r.date; });
      // Expand a bit for padding
      minDate = new Date(minDate.getTime() - 5*1000);
      maxDate = new Date(maxDate.getTime() + 5*1000);

      // Graph
      var graph = buildGraph(rows);
      var layout = layoutNodes(graph.nodes, graph.links);

      // Resize outer svg height if needed
      d3.select('svg').attr('height', layout.totalHeight + plotYOffset + margin.top + margin.bottom);

      // Prepare color domain as unique syscalls using loaded colors, ordered by frequency
      var syscallSet = d3.set(rows.map(function(r){ return r.syscall; })).values();
      // Sort by frequency (most frequent first) using the frequency data from colors.json
      syscallSet.sort(function(a, b) {
        var freqA = colors.systemCallFrequency[a] || 0;
        var freqB = colors.systemCallFrequency[b] || 0;
        return freqB - freqA; // Descending order (most frequent first)
      });
      // Use loaded colors for system calls
      colorScale = d3.scale.ordinal()
        .domain(syscallSet)
        .range(syscallSet.map(function(type) { 
          return colors.systemCalls[type] || colors.systemCalls['read']; // fallback to read color
        }));

      // Width scale: if multiple events at the exact same second for a pair, aggregate would be better
      arcWidthScale.domain([1, 5]);

      gState.nodes = graph.nodes;
      gState.links = graph.links;
      gState.timeDomain = [minDate, maxDate];

      // Precompute globalMaxBinTotal using time-based binning (120 bins per minute)
      (function computeGlobalMax() {
    var t0 = minDate.getTime();
    var t1 = maxDate.getTime();
    
    // Calculate time range in seconds and round up to nearest half or full minute
    var timeRangeSeconds = (t1 - t0) / 1000;
    var minutes = Math.ceil(timeRangeSeconds / 30) * 0.5; // Round up to nearest 0.5 minute
    if (minutes < 1) minutes = 1; // Minimum 1 minute
    
    // Calculate number of bins: 120 bins per minute
    var totalBins = Math.ceil(minutes * 120);
    
    var step = (t1 - t0) / totalBins;
    if (step <= 0) step = 1;
    var bins = d3.range(totalBins).map(function(i){
      return { countsByNode: {} , t0: new Date(t0 + i*step), t1: new Date(t0 + (i+1)*step)};
    });
    graph.links.forEach(function(l){
      var tt = l.date.getTime();
      if (tt < t0 || tt > t1) return;
      var bi = Math.min(totalBins - 1, Math.max(0, Math.floor((tt - t0)/step)));
      var bin = bins[bi];
      var nodeId = graph.nodes[l.source].id;
      if (!bin.countsByNode[nodeId]) bin.countsByNode[nodeId] = 0;
      bin.countsByNode[nodeId] += 1;
    });
    var maxTotal = 1;
    bins.forEach(function(b){
      for (var nid in b.countsByNode) {
        if (b.countsByNode[nid] > maxTotal) maxTotal = b.countsByNode[nid];
      }
    });
    gState.globalMaxBinTotal = maxTotal;
  })();

      draw(graph.nodes, graph.links, gState.timeDomain);
      buildLegend(syscallSet);
      buildSizeLegend();
      
      // Initialize bundling visualization
      initBundling();
      processBundlingData(rows);
      
      // Update arcWidthScale domain based on bundling data before rendering
      var bundlingMaxCount = d3.max(bundlingEdges, function(d) { return d.count; }) || 1;
      arcWidthScale.domain([1, Math.max(1, bundlingMaxCount)]);
      
      renderBundling(); // Initial render of bundling visualization
      
      // Setup filter toggle buttons and popup after a short delay to ensure DOM is ready
      setTimeout(function() {
        setupFilterToggleButtons();
        setupBundlingPopup();
        positionButtonsBelowLegend();
        // Force initial layout without needing manual zoom/resize
        onResize();
      }, 100);
    });
}

function init() {
  // Load colors first (once); datasets are then loaded on demand
  d3.json('colors.json', function(error, colors) {
    if (error) { console.error(error); return; }
    gState.colors = colors;

    // Apply background and text colors from loaded colors
    d3.select('body')
      .style('background-color', colors.ui.background)
      .style('color', colors.ui.text);

    // Wire dataset dropdown: reload + full re-render on change
    d3.select('#datasetSelect').on('change', function() {
      loadDataset(this.value);
    });

    // Bind resize once (loadDataset reuses gState via onResize)
    window.addEventListener('resize', onResize);

    // Initial load of the default dataset (matches the pre-existing behavior)
    loadDataset(CSV_PATH);
  });
}

function setupFilterToggleButtons() {
  d3.select('#showAllFilterBtn')
    .style('background-color', '#ffffff')
    .style('color', '#000000')
    .on('click', function(){
      callStatusFilter = 'all';
      // Remove active class from all buttons
      d3.selectAll('#filterToggleBtns button').classed('active', false);
      // Add active class to clicked button
      d3.select(this).classed('active', true);
      redrawArcs();
      renderBundling();
    });

  d3.select('#unfinishedFilterBtn')
    .style('background-color', gState.colors.statusColors.unfinished)
    .style('color', '#ffffff')
    .on('click', function(){
      callStatusFilter = 'unfinished';
      // Remove active class from all buttons
      d3.selectAll('#filterToggleBtns button').classed('active', false);
      // Add active class to clicked button
      d3.select(this).classed('active', true);
      redrawArcs();
      renderBundling();
    });

  d3.select('#resumedFilterBtn')
    .style('background-color', gState.colors.statusColors.resumed)
    .style('color', '#ffffff')
    .on('click', function(){
      callStatusFilter = 'resumed';
      // Remove active class from all buttons
      d3.selectAll('#filterToggleBtns button').classed('active', false);
      // Add active class to clicked button
      d3.select(this).classed('active', true);
      redrawArcs();
      renderBundling();
    });
    
  d3.select('#normalFilterBtn')
    .style('background-color', gState.colors.statusColors.normal)
    .style('color', '#ffffff')
    .on('click', function(){
      callStatusFilter = 'normal';
      // Remove active class from all buttons
      d3.selectAll('#filterToggleBtns button').classed('active', false);
      // Add active class to clicked button
      d3.select(this).classed('active', true);
      redrawArcs();
      renderBundling();
    });
}

function positionButtonsBelowLegend() {
  var legend = document.getElementById('legend');
  var buttonContainer = document.getElementById('filterToggleBtns');
  var bundlingButton = document.getElementById('bundlingTriggerBtn');
  
  if (legend && buttonContainer) {
    var legendRect = legend.getBoundingClientRect();
    var sizeLegend = document.getElementById('sizeLegend');
    var topAnchor = legendRect.bottom + 8;
    var leftAnchor = legendRect.left;
    if (sizeLegend) {
      var srect = sizeLegend.getBoundingClientRect();
      topAnchor = srect.bottom + 8;
      leftAnchor = srect.left;
    }
    
    // Position filter buttons directly below the size legend (if present)
    buttonContainer.style.position = 'fixed';
    buttonContainer.style.top = (topAnchor + window.scrollY) + 'px';
    buttonContainer.style.left = leftAnchor + 'px';
    buttonContainer.style.right = 'auto'; // Remove right positioning
    
    // Position bundling button BELOW the status buttons
    if (bundlingButton) {
      var brect = buttonContainer.getBoundingClientRect();
      bundlingButton.style.position = 'fixed';
      bundlingButton.style.top = (brect.bottom + 8) + 'px';
      bundlingButton.style.left = brect.left + 'px';
      bundlingButton.style.right = 'auto';
    }
  }
}

// Popup functionality
function showBundlingPopup() {
  var popup = d3.select('#bundlingPopup');
  popup.style('display', 'flex');
  
  // Update bundling SVG dimensions for popup
  if (bundlingSvg) {
    var popupContent = d3.select('.bundling-popup-content');
    var popupWidth = 600; // Fixed width
    var popupHeight = 480; // Fixed height minus even smaller header (500 - 20)
    
    bundlingSvg.attr('width', popupWidth)
               .attr('height', popupHeight);
    
    // Update bundling transform
    bundlingG.attr('transform', 'translate(' + (popupWidth/2) + ',' + (popupHeight/2) + ')');
    
    // Re-render bundling with new dimensions
    renderBundling();
  }
}

function hideBundlingPopup() {
  var popup = d3.select('#bundlingPopup');
  popup.style('display', 'none');
}

function setupBundlingPopup() {
  // Setup trigger button
  d3.select('#bundlingTriggerBtn')
    .on('click', showBundlingPopup);
  
  // Setup close button
  d3.select('#bundlingPopupClose')
    .on('click', hideBundlingPopup);
  
  // Note: Removed click-outside-to-close since there's no overlay
  
  // Close popup with Escape key
  d3.select(window)
    .on('keydown.bundlingPopup', function() {
      var e = d3.event;
      if (e.key === 'Escape' && d3.select('#bundlingPopup').style('display') !== 'none') {
        hideBundlingPopup();
      }
    });
  
  // Setup drag functionality
  var popupContent = d3.select('.bundling-popup-content');
  var isDragging = false;
  var dragOffset = { x: 0, y: 0 };
  
  // Make the header draggable
  var popupHeader = d3.select('.bundling-popup-header');
  
  popupHeader.call(d3.behavior.drag()
    .on('dragstart', function() {
      isDragging = true;
      var rect = popupContent.node().getBoundingClientRect();
      var mouseX = d3.event.sourceEvent.clientX;
      var mouseY = d3.event.sourceEvent.clientY;
      dragOffset.x = mouseX - rect.left;
      dragOffset.y = mouseY - rect.top;
      d3.event.sourceEvent.preventDefault();
    })
    .on('drag', function() {
      if (isDragging) {
        var mouseX = d3.event.sourceEvent.clientX;
        var mouseY = d3.event.sourceEvent.clientY;
        var x = mouseX - dragOffset.x;
        var y = mouseY - dragOffset.y;
        
        // Keep popup within viewport bounds
        var popupWidth = 600;
        var popupHeight = 500;
        var maxX = window.innerWidth - popupWidth;
        var maxY = window.innerHeight - popupHeight;
        
        x = Math.max(0, Math.min(x, maxX));
        y = Math.max(0, Math.min(y, maxY));
        
        popupContent.style('left', x + 'px')
                   .style('top', y + 'px')
                   .style('transform', 'none');
      }
    })
    .on('dragend', function() {
      isDragging = false;
    }));
}


init();

