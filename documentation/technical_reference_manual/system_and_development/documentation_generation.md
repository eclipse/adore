# Documentation Generation
ADORe provides tools to generate all of the documentation detailed in the 
[documentation](documentaiton.md) readme.

## Usage: Generation
1. cd to the adore documentation directory:
```bash
cd adore/documentation
```
2. Call the build target:
```bash
make build
```

## Usage: Publication
Assuming you have properly forked the adore repo you can use the Publication
target to publish the documents to your personal gh-pages with:
```bash
make publish
```
