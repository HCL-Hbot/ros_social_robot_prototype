#!/bin/bash

# Run Sass in watch mode in background
npm run sass-watch &

# Forward all arguments to Electron
npx electron . "$@"
