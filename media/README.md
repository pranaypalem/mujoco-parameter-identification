# Media Files

This directory contains generated media files from the MuJoCo simulations.

## Contents

- `*.gif` - Generated GIF animations from simulations
- `*.mp4` - Video files (if generated)
- `*.png` - Plot images and figures

## Generating GIFs

To generate a GIF from the simulation, run:

```bash
make generate-gif
```

Or directly:

```bash
python src/gif_generator.py
```