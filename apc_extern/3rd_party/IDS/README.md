IDS camera drivers
==================

Linux drivers for the IDS cameras. To get the password for the archive email `raambrus` at `kth` dot `se`. 

Read the file ```Linux_ReadMe_4.72_EN.txt``` for instructions on how to install the drivers. 

After installation, run the following to start the uEye daemon (you need to be root):
note: depending on your system settings you might need to start the daemon only once.

```
/etc/init.d/ueyeusbdrc start #usb connection
/etc/init.d/ueyeethdrc start #eth connection
```

To stop the daemon run:

```
/etc/init.d/ueyeusbdrc stop #usb connection
/etc/init.d/ueyeethdrc stop #eth connection
```
