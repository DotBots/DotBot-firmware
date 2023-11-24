# ED25519 signature sample application

This application shows how to use the ED25519 asymmetric signature library.

The main.c contains a private key used to sign a message and public key used to
verify the signature of the message.

The [signature.py](../../dist/scripts/crypto/signature.py)
python script is provided so a different key
pair can be used to sign and verify. The script can be used to generate an new
key pair and the signature of the test message defined in the application.

The script requires the
[cryptography package](https://pypi.org/project/cryptography/) to be installed:

```
pip install -r dist/scripts/crypto/requirements.txt
```

Run the following command to generate the header files containing the keys
and the expected signature:

```
python dist/scripts/crypto/signature.py
```
