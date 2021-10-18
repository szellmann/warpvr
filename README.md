Remote Volume Rendering with a Decoupled, Ray-Traced Display Phase
==================================================================

Sample code for the paper S. Zellmann (2021), "Remote Volume Rendering with a Decoupled, Ray-Traced Display Phase", STAG: Smart Tools and Applications in Graphics (2021)

<img src="/img/1.jpg" width="270" /><img src="/img/2.jpg" width="270" /><img src="/img/3.jpg" width="270" />

On Ubuntu 20.04 (tested), compile this code as usual (`git clone` with
`--recursive`, `cmake`, `make`, Release mode and C++11 enabled, NVIDIA CUDA in
a reasonably new version).  You'll likely run into some trouble with swig if
you have that installed; just unset the `SWIG_` related variables in the cmake
cache then.

Run this code by starting the server process from the build directory:

`./server /path/to/rawVolumeFile`

Then start the client from the build directory, too:

`./client`

The server should be able to load most of the raw files from Pavol Klacansky's
website: https://klacansky.com/open-scivis-datasets/

The client understands the following command line options:

```
Usage:
   ./client [OPTIONS]

Options:
   -bgcolor               Background color
   -fullscreen            Full screen window
   -height=<ARG>          Window height
   -width=<ARG>           Window width
```

(bgcolor is RGB and each component is in [0.0..1.0]).

# Citation

If this sample code is useful to you I'd appreciate if you would cite the paper:

```
@inproceedings {zellmann:2021,                                                                
booktitle = {STAG: Smart Tools and Applications in Graphics (2021)},                  
editor = {P. Frosini and D. Giorgi and S. Melzi and E. Rodola},                                               
title = {Remote Volume Rendering with a Decoupled, Ray-Traced Display Phase},                                 
author = {Zellmann, Stefan},                      
year = {2021},                                                                                
publisher = {The Eurographics Association},                                                                 
}
```

# License

This sample code is licensed under the MIT License (MIT)
