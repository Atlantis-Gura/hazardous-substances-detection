# Hazardous Substances Detection System

🚧 **Work in Progress** 🚧

The complete codebase will be uploaded after the associated paper is published and made public.

## Repository Overview

This repository contains the code and data collection software for a hazardous substances detection system, as described in the research paper. The system utilizes TinyML and specialized sensors to provide real-time detection and alerts for dangerous gases and substances.

This hazardous substances detection system is designed to enhance workplace safety by providing early warning of potentially dangerous environmental conditions. It combines mems mos sensor technology with machine learning algorithms to accurately identify and quantify hazardous substances in real-time.

## Repository Structure

The repository is organized as follows:

github-repo-root
├── src
│   ├── arduino_sketches
│   │   ├── main
│   │   │   ├── main.ino
│   │   │   └──_icons
│   │   │       └── logo.c
│   │   └── gasDataCollector
│   │       └── gasDataCollector.ino
│   └── libs
└── tools
    └── lcd-image-converter

### `src` Directory

The `src` directory contains the core source code and libraries:

- `arduino_sketches`: Contains Arduino sketches for the project
  - `main`: The primary system code
    - `main.ino`: The main code file for the system
    - `_icons/logo.c`: System logo icon file
  - `gasDataCollector`: Program used to collect data for training the TinyML model
    - `gasDataCollector.ino`: Arduino sketch for data collection
- `libs`: Contains all third-party libraries required for the project

### `tools` Directory

The `tools` directory contains third-party software used for development and testing:

- `lcd-image-converter`: Tool for LCD image conversion

## Installation and Usage

1. Clone this repository:
`git clone <https://github.com/Atlantis-Gura/hazardous-substances-detection.git>`

2. Install the required libraries:

   - Open Arduino IDE
   - Go to "Sketch" > "Include Library" > "Add .ZIP Library"
   - Navigate to the `src/libs` directory in the cloned repository
   - Select and install all .zip files in this directory

   Note: Some libraries may also be available through the Arduino IDE Library Manager. For specific version information, please refer to the library versions listed in the research paper.

3. Open and upload the main program:

   - Open `src/arduino_sketches/main/main.ino` in Arduino IDE
   - Select the correct board and port
   - Click the upload button

   For data collection, upload `src/arduino_sketches/gasDataCollector/gasDataCollector.ino` to your Arduino-compatible board.

## License

This project is primarily licensed under the Apache License 2.0.

- All original code and modifications created for this project are licensed under the Apache License 2.0. See [LICENSE-APACHE](LICENSE-APACHE) for the full text.

### Third-Party Code and Libraries

This project includes code and libraries from third-party sources, each with its own license:

1. **bhopal84 Project**:
   A small portion of code is derived from the [bhopal84](https://github.com/ronibandini/bhopal84) project, which is licensed under the MIT License. It's important to note that this code has been extensively refactored and modified for use in this project. The original concepts and basic structure remain, but the implementation has been significantly altered. See [LICENSE-MIT](LICENSE-MIT) for details of the original MIT license.

2. **Third-Party Libraries**:
   This project includes several third-party libraries, each with its own license. These libraries can be found in the `src/libs` directory.

   It is the user's responsibility to comply with the license terms of these third-party libraries when using or distributing this project.

Please refer to the individual library files or their respective documentation for their specific license terms.

For additional information about included software and attributions, please see the [NOTICE](NOTICE) file.

## Contact

For questions or feedback, please open an issue in this repository or contact the project maintainer.

---
For more detailed information about the research and findings, please refer to the [research paper](research_paper).
