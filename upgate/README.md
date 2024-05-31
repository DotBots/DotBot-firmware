# upGate

Over-the-air dynamic reconfiguration of FPGA library and example application.

Use the [dotbot-upgate.py](../dist/scripts/upgate/dotbot-upgate.py) Python script to
reconfigure an FPGA with a new bitstream. For better performances, the bitstream
can be sent compressed (Gzip andn LZ4 compression methods available).
Among different common Python packages, this script requires the
[pydotbot](https://pypi.org/project/pydotbot/) package to be installed on the
system.

To install all the Python dependencies (pydotbot, cryptography, click and tqdm), run:

```
pip install -r dist/scripts/otap/requirements.txt
```
