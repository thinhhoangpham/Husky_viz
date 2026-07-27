# park.world — notes

## `Untitled2` has a dead mesh path — deliberately left broken

`natural_environments_ros/natural_enviroment/worlds/park.world` defines a model
named `Untitled2` whose visual and collision geometry both point at:

    /home/a/Desktop/modelos_mundo_dataset/terreno_dataset.dae

That is the original author's home directory and does not exist here.

**Decision (2026-07-19): leave it broken. Do not "fix" this path.**

Reason — `Untitled2` is unused junk, not terrain. Its saved pose in the world's
`<state>` section is:

    -129.711 -58.6897 -1.75917e+08

i.e. z ≈ −1.76 x 10^8 m — about 176,000 km below the world, roughly half the
distance to the Moon. The author dragged it out of the scene rather than
deleting it. This is the hand-editing artifact noted in
`park_1_topic_breakdown.md:16`.

**Consequence: Gazebo will log an error that it cannot find this mesh when
loading park.world. That error is EXPECTED and harmless.** Do not treat it as a
failed load.

## The real ground is `parque` and `camino_parque`

Verified by XML parse of the world file:

| Model | Collisions | Visuals | Mesh | Resolves? |
|---|---|---|---|---|
| `parque` | 1 | 1 | `model://terreno_parque/terreno_parque.dae` | yes |
| `camino_parque` | 1 | 1 | `model://camino_parque/camino_parque.dae` | yes |
| `Untitled2` | 2 | 2 | dead absolute path | no |

`parque` (saved pose z = +2.99) carries real collision geometry — it is the
surface the robot drives on. All 14 `model://` URIs in park.world resolve
against `/Volumes/Extreme Pro/Husky viz/models/` (97 model dirs, the extracted
`models.zip` from `natural_environments_ros/readme.txt:11`).

## Don't be fooled by the duplicate terrain meshes

Two different local copies of `terreno_dataset.dae` exist:

    models/terreno_dataset/terreno_dataset.dae   138,714,667 bytes
    models/terrain/terreno_dataset.dae           363,399,538 bytes

Different checksums, different sizes. Neither is "the right one" — the model
that references them is unused, so there is nothing to choose between.
