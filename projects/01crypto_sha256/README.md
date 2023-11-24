# SHA256 hash computation sample application

This application shows how to use the SHA256 hashing method.

The [sha256.py](../../dist/scripts/crypto/sha256.py)
python script is provided to show how to produce the
SHA256 hash of a message. The result is stored in a binary file.

The script requires the
[cryptography package](https://pypi.org/project/cryptography/) to be installed:

```
pip install -r dist/scripts/crypto/requirements.txt
```

Run the following command to generate the expected sha256:

```
python dist/scripts/crypto/sha256.py
```
