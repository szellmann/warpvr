Augmenting Image Warping-Based Remote Volume Rendering with Ray Tracing
=======================================================================

Sample code for the paper S. Zellmann (2020), "Augmenting Image Warping-Based
Remote Volume Rendering with Ray Tracing" (Preprint under:
https://arxiv.org/abs/2006.14726)

<img src="/img/1.jpg" width="270" /><img src="/img/2.jpg" width="270" /><img src="/img/3.jpg" width="270" />

This paper hasn't been peer-reviewed, nor submitted to a journal (yet).

On Ubuntu 18.04 (tested), compile this code as usual (`git clone` with
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

Currently, the paper is available as a preprint (I might at some point consider
to add to it and submit to a conference or journal as a short paper, but so far
there are no concrete plans); in the meantime, if you want to reference this
work in your own paper, please cite it as:

```
@misc{zellmann2020augmenting,
      title={Augmenting Image Warping-Based Remote Volume Rendering with Ray Tracing},
      author={Stefan Zellmann},
      year={2020},
      eprint={2006.14726},
      archivePrefix={arXiv},
      primaryClass={cs.GR}
}
```

# License

This sample code is licensed under the MIT License (MIT)
