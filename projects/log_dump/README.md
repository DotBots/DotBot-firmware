# Log dump companion

This application looks for log data stored on flash and dumps them to stdio.

## Usage

To retrieve the logs, use Segger Embedded Studio as follows:
1. Start a debug session (F5) and run the program (F5 again)
2. Click inside the Debug Terminal window so it has focus, then press "s". The
program will print the logs (or an error message if none is found and the version is
incompatible)
3. Right click on the debug terminal and choose "Export to text editor". This
will open a new tab with the output content. Remove what's not in CSV format and
save the file on your disk for later analysis.
