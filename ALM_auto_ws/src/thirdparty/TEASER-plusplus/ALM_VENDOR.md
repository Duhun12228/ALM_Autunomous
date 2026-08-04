# TEASER++ vendoring record

- Upstream: https://github.com/MIT-SPARK/TEASER-plusplus
- Tag: `v2.0`
- Commit: `d19ce8c8fc58f43fcd1e19a9cfd6ed32717ef3cf`
- License: MIT (`LICENSE`)

Local integration changes:

- Disable unused PLY I/O, Python, MATLAB, documentation, and test targets in the ALM build.
- Download GoogleTest only when `BUILD_TESTS=ON`.
- Pin the PMC dependency to `a2dfd612a501bca83c47206255dbbff619481f97`.
- Qualify `std::vector` in `teaser/src/graph.cc` for compatibility with the pinned PMC headers.

The runtime uses only `teaserpp::teaser_registration`; FPFH extraction and matching use the
workspace's PCL 1.12 installation.
