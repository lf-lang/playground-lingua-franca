# VL53L1X guideline

To handle the i2c bus  
```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install python3-pip
sudo pip3 install --upgrade setuptools
cd ~
sudo pip3 install --upgrade adafruit-python-shell
wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/raspi-blinka.py
sudo python3 raspi-blinka.py
```

pin connection and above instructions  
https://learn.adafruit.com/adafruit-vl53l1x/python-circuitpython

npm module
```
npm install vl53l1x
```
https://github.com/williamkapke/vl53l1x  
https://www.npmjs.com/package/vl53l1x

# Run a TypeScript-targeted LF file that requires a new node module not included in the default package.json

1. compile the LF file

2. go to the src-gen/<main-reactor-name>/ or fed-gen/<LF-file-name>/src-gen/<reactor-name> folder

3. copy the package.json file to the src/ folder.

4. expand dependencies with an appropriate module name
ex) When I want to use the MQTT module,
    "dependencies": {
        "@lf-lang/reactor-ts": "0.2.0",
        ...
        "rimraf": "^3.0.2",
        "mqtt": "^4.3.7"
    },
    ...

- To check the version of the module, you can run npm install <module-name> in any folder and check the generated package.json in that folder.

5. (optional) if you are trying to compile federation files, copy the package.json file to the fed-gen/<LF-file-name>/src, too.

6. Recompile the file, the package.json in the src-gen folder or fed-gen folder now include a new dependency.