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

## Usage: Publication (to gh-pages)
Assuming you have properly forked the adore repo you can use the Publication
target to publish the documents to your personal gh-pages with:
```bash
make publish
```

You have to enable gh-pages on the `docs` directory in order for the publication
to be active. Visit `https://github.com/<username>/adore/settings/pages` to
configure gh-pages. 

## Usage: Serving local copy
You can build and serve the documentation locally by running the provide `make
serve` target. Navigate to the documentation directory and run the following:
```bash
cd adore/documentation
make serve
```

Once built the documents will be available at
[http://localhost 🔗](http://localhost) 
