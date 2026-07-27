// Global variables
let rawData = [];
let processedData = [];
let colors = {};

// Load colors from colors.json
async function loadColors() {
    try {
        const response = await fetch('colors.json');
        const colorData = await response.json();
        colors = colorData.systemCalls;
        console.log('Loaded colors:', colors);
    } catch (error) {
        console.error('Error loading colors.json:', error);
        // Fallback colors if colors.json is not available
        colors = {
            'clock_gettime': '#8dd3c7',
            'futex': '#ffffb3', 
            'write': '#bebada',
            'select': '#fb8072',
            'recvfrom': '#80b1d3',
            'read': '#fdb462',
            'brk': '#b3de69',
            'clock_nanosleep': '#fccde5',
            'close': '#d9d9d9',
            'poll': '#bc80bd',
            'times': '#ccebc5',
            'epoll_ctl': '#ffed6f',
            'sendto': '#a8e6cf',
            'openat': '#ffb3ba',
            'fstat': '#bae1ff',
            'shutdown': '#ffdfba'
        };
    }
}

// Default dataset (first page load reproduces the original behavior)
const DEFAULT_CSV = 'umdhusky-data_collection-syscall-auto-gps-no-cam-processed.csv';

// Load and process data for the given CSV file
async function loadData(csvPath = DEFAULT_CSV) {
    try {
        const response = await fetch(csvPath);
        const csvText = await response.text();
        
        // Parse CSV data
        const lines = csvText.split('\n');
        const headers = lines[0].split(',');
        
        rawData = lines.slice(1)
            .filter(line => line.trim())
            .map(line => {
                const values = line.split(',');
                const row = {};
                headers.forEach((header, i) => {
                    row[header.trim()] = values[i] ? values[i].trim() : '';
                });
                return row;
            })
            .filter(row => row['Epoch Time'] && row['Node'] && row['System Call']);
        
        console.log(`Loaded ${rawData.length} system call records`);
        processData();
    } catch (error) {
        console.error('Error loading data:', error);
        document.getElementById('chart').innerHTML = '<p>Error loading data. Please make sure the CSV file is accessible.</p>';
    }
}

// Process data for ridge plot
function processData() {
    // Calculate time range in seconds
    const timeMin = d3.min(rawData, d => +d['Epoch Time']);
    const timeMax = d3.max(rawData, d => +d['Epoch Time']);
    const timeRangeSeconds = timeMax - timeMin;
    
    // Round up to nearest half or full minute
    const timeRangeMinutes = Math.ceil(timeRangeSeconds / 60);
    const roundedTimeRangeSeconds = timeRangeMinutes * 60;
    
    // Use 120 bins per minute (0.5-second bins)
    const binsPerMinute = 120;
    const binSizeSeconds = 60 / binsPerMinute; // 0.5 seconds
    const totalBins = Math.ceil(roundedTimeRangeSeconds / binSizeSeconds);
    
    // Create time bins
    const timeBins = [];
    for (let i = 0; i < totalBins; i++) {
        timeBins.push(timeMin + (i * binSizeSeconds));
    }
    
    // Group data by system call and time bin
    const groupedData = d3.group(rawData, d => d['System Call'], d => {
        const time = +d['Epoch Time'];
        const binIndex = Math.floor((time - timeMin) / binSizeSeconds);
        return timeBins[binIndex] || timeBins[timeBins.length - 1];
    });
    
    // Create series data for each system call
    const series = [];
    const systemCallNames = Array.from(groupedData.keys()).sort();
    
    systemCallNames.forEach(systemCallName => {
        const systemCallData = groupedData.get(systemCallName);
        const values = timeBins.map(timeBin => {
            const binData = systemCallData.get(timeBin);
            if (!binData) return 0;
            
            return binData.length;
        });
        
        series.push({
            name: systemCallName,
            values: values
        });
    });
    
    processedData = {
        series: series,
        timeBins: timeBins,
        timeMin: timeMin,
        timeMax: timeMax,
        binSizeSeconds: binSizeSeconds,
        totalBins: totalBins,
        timeRangeSeconds: timeRangeSeconds,
        roundedTimeRangeSeconds: roundedTimeRangeSeconds
    };
    
    createRidgePlot();
}

// Create the ridge plot visualization
function createRidgePlot() {
    // Clear previous chart and any stale tooltip from a prior render
    d3.select('#chart').selectAll('*').remove();
    d3.select('body').selectAll('div.tooltip').remove();

    if (!processedData.series.length) {
        document.getElementById('chart').innerHTML = '<p>No data to display.</p>';
        return;
    }
    
    const { series, timeBins, timeMin, timeMax } = processedData;
    
    // Chart dimensions
    const overlap = 8;
    const width = 1400;
    const height = series.length * 25 + (overlap * series.length);
    const marginTop = 20;
    const marginRight = 20;
    const marginBottom = 20;
    const marginLeft = 100;
    
    // Create scales
    const x = d3.scaleTime()
        .domain([timeMin * 1000, timeMax * 1000]) // Convert to milliseconds for Date
        .range([marginLeft, width - marginRight]);
    
    const y = d3.scalePoint()
        .domain(series.map(d => d.name))
        .range([marginTop + overlap * series.length, height - marginBottom]);
    
    const z = d3.scaleLinear()
        .domain([0, d3.max(series, d => d3.max(d.values))]).nice()
        .range([0, -overlap * y.step()]);
    
    // Create area generator
    const area = d3.area()
        .curve(d3.curveBasis)
        .defined(d => !isNaN(d))
        .x((d, i) => x(timeBins[i] * 1000))
        .y0(0)
        .y1(d => z(d));
    
    const line = area.lineY1();
    
    // Create SVG
    const svg = d3.select('#chart')
        .append('svg')
        .attr('width', width)
        .attr('height', height)
        .attr('viewBox', [0, 0, width, height])
        .attr('style', 'max-width: 100%; height: auto;');
    
    // Add axes
    svg.append('g')
        .attr('transform', `translate(0,${height - marginBottom})`)
        .call(d3.axisBottom(x)
            .ticks(width / 80)
            .tickFormat(d3.timeFormat('%H:%M:%S'))
            .tickSizeOuter(0));
    
    svg.append('g')
        .attr('transform', `translate(${marginLeft},0)`)
        .call(d3.axisLeft(y)
            .tickSize(0)
            .tickPadding(4)
            .tickFormat(d => d)) // Show system call names
        .call(g => g.select('.domain').remove());
    
    
    // Create ridge groups
    const group = svg.append('g')
        .selectAll('g')
        .data(series)
        .join('g')
        .attr('transform', d => `translate(0,${y(d.name) + 1})`);
    
    // Add filled areas
    group.append('path')
        .attr('fill', d => colors[d.name] || '#999')
        .attr('fill-opacity', 0.6)
        .attr('d', d => area(d.values));
    
    // Add top lines
    group.append('path')
        .attr('fill', 'none')
        .attr('stroke', d => colors[d.name] || '#999')
        .attr('stroke-width', 1.5)
        .attr('d', d => line(d.values));
    
    // Add tooltips
    const tooltip = d3.select('body').append('div')
        .attr('class', 'tooltip')
        .style('position', 'absolute')
        .style('background', 'rgba(0, 0, 0, 0.8)')
        .style('color', 'white')
        .style('padding', '8px')
        .style('border-radius', '4px')
        .style('font-size', '12px')
        .style('pointer-events', 'none')
        .style('opacity', 0);
    
    // Add interactive areas for tooltips
    group.append('path')
        .attr('fill', 'transparent')
        .attr('d', d => area(d.values))
        .on('mouseover', function(event, d) {
            tooltip.style('opacity', 1);
        })
        .on('mousemove', function(event, d) {
            const [mouseX] = d3.pointer(event, this);
            const timeValue = x.invert(mouseX);
            const binIndex = Math.round((timeValue - timeMin * 1000) / (processedData.binSizeSeconds * 1000));
            const value = d.values[binIndex] || 0;
            
            tooltip
                .html(`
                    <strong>${d.name}</strong><br/>
                    Time: ${d3.timeFormat('%H:%M:%S')(new Date(timeValue))}<br/>
                    Count: ${value.toFixed(0)}
                `)
                .style('left', (event.pageX + 10) + 'px')
                .style('top', (event.pageY - 10) + 'px');
        })
        .on('mouseout', function() {
            tooltip.style('opacity', 0);
        });
}


// Initialize the visualization
async function initialize() {
    await loadColors();

    // Reload + fully re-render when the user picks a different dataset
    const selector = document.getElementById('datasetSelect');
    if (selector) {
        selector.addEventListener('change', () => {
            loadData(selector.value);
        });
    }

    // Initial load: default dataset (or whatever the selector currently shows)
    await loadData(selector ? selector.value : DEFAULT_CSV);
}

initialize();
