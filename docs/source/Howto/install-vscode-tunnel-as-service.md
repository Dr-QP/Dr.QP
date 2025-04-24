# Install VSCode tunnel as service

## On RPi or Apple Silicon

```
curl -Lk 'https://code.visualstudio.com/sha/download?build=stable&os=cli-alpine-arm64' --output vscode_cli.tar.gz
tar -xf vscode_cli.tar.gz
./code tunnel service install
```

## On x86

```
curl -Lk 'https://code.visualstudio.com/sha/download?build=stable&os=cli-alpine-x64' --output vscode_cli.tar.gz
tar -xf vscode_cli.tar.gz
./code tunnel service install
```
