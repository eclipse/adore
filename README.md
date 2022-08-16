# ADORe Github Pages Documentation Tool
This branch contains a tool for generating the ADORe  Github Pages documentation static site.

## Usage
1. Clone the adore repository and checkout out this branch
```bash
git clone git@github.com:eclipse/adore.git 
cd adore
git checkout gh-Pages
```

2. Modify the config.env file and change the GIT_BRANCH variable to what ever source documentation branch you want to use.

3. Run the provided shell script:
```bash
make
```
