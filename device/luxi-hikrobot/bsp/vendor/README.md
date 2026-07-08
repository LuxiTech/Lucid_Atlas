# Vendored MVS Python Bindings

This directory vendors `MvImport/*.py` from the `hikrobot-sdk` Python package
version `0.1.0`.

Why this exists:

- The installed runtime package under `/opt/MVS` provides the shared library
  `libMvCameraControl.so`, but it does not ship the official C header files in
  this workspace.
- We still need stable structure definitions and function signatures to build a
  reusable local BSP layer.
- The vendored `MvImport` files provide a ctypes mapping for the same SDK ABI
  and let us keep development local to this repository.

Reference source:

- PyPI package: `hikrobot-sdk==0.1.0`
- Package metadata declares `License: MIT`

This vendor layer is only the low-level binding. Project code should import the
high-level wrapper from `luxitech.bsp.mvs_camera` instead of calling the
vendored files directly.
