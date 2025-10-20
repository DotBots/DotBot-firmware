# DotBot: easy-to-use micro-robot for education and research purposes

[![CI][ci-badge]][ci-link]
[![Documentation][doc-badge]][doc-link]
[![License][license-badge]][license-link]

## Overview

This repository contains the source code for firmwares usable with the DotBot
hardware.

<video src="https://github.com/user-attachments/assets/d78320cb-960c-483d-a5a1-f46c38e8455e" type="video/mp4" controls width="100%">
</video>

## Related projects

The DotBots ecosystem provides Python
[PyDotBot](https://github.com/DotBots/PyDotBot) to interacts with a nRF DK board
used a gateway.

The [DotBot-libs](https://github.com/DotBots/DotBot-libs) repository provides
low-level peripheral drivers and libraries to facilitate the development of
applications for the DotBot platform.

The DotBots hardware design are published on GitHub at
[https://github.com/DotBots/DotBot-hardware](https://github.com/DotBots/DotBot-hardware).

## Get the code

This repository uses [DotBot-libs](https://github.com/DotBots/DotBot-libs) as a
[Git submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules).
To clone the repository use the following command:

```
git clone --recurse-submodules https://github.com/DotBots/DotBot-firmware
```

## Building firmwares

The source code of the different applications available in this repository can be built using
[SEGGER Embedded Studio for ARM](https://www.segger.com/downloads/embedded-studio).
In SEGGER embedded studio, use the package manager
(available in menu Tools > Package manager) to install the CMSIS 5 CMSIS-CORE,
CMSIS-DSP and nRF packages.

For details on SEGGER Embedded Studio, read the
[online documentation](https://studio.segger.com/index.htm?https://studio.segger.com/home.htm).

[ci-badge]: https://github.com/DotBots/DotBot-firmware/workflows/CI/badge.svg
[ci-link]: https://github.com/DotBots/DotBot-firmware/actions?query=workflow%3ACI+branch%3Amain
[doc-badge]: https://readthedocs.org/projects/dotbot-firmware/badge/?version=latest
[doc-link]: https://dotbot-firmware.readthedocs.io/en/latest
[license-badge]: https://img.shields.io/github/license/DotBots/DotBot-firmware
[license-link]: https://github.com/DotBots/dotbot-firmware/blob/main/LICENSE.txt
