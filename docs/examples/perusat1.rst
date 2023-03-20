.. _perusat1:

PeruSat-1
---------

PeruSat-1 provides exact linescan camera models and RPC-approximated
camera models in separate files. The names for these start with
"DIM" and "RPC", respectively, and end with ".XML".

If desired to use the exact model, the stereo command is::

    parallel_stereo -t perusat --stereo-algorithm asp_mgm \
        left.tif right.tif left.xml right.xml results/run

For the RPC model the option ``-t rpc`` should be used and the correct
camera files should be passed in. See also :numref:`rpc`.

If the ``-t`` option is not specified, it will be auto-guessed
based on the content of the camera files provided as inputs.

For PeruSat-1 exact linescan camera models the atmospheric correction and
velocity aberration corrections (:numref:`sensor_corrections`) are
disabled, as these decrease somewhat the agreement with the RPC
models. 

DEMs created with the exact and RPC models differ by a systematic
vertical shift of about 15 meters for unknown reasons, even though the
intersection error maps are very similar. Nothing in the sensor manual
or camera metadata suggests the cause of this. The ``pc_align`` tool
can be used to reduce this discrepancy. The mean absolute
difference of the (full-image extent) aligned DEMs is about 0.17
meters.

See :numref:`nextsteps` for a discussion about various speed-vs-quality choices.
