# Vagrantfile Readme
In the ADORe tools directory you will find a Vagrantfile. This vagrant 
context/virtual machine comes pre-installed with docker and make in order to 
build ADORe.

## Getting Started
1. install Virtualbox
2. install vagrant
3. run vagrant
```bash
vagrant up && vagrant ssh
```

## Cleaning Up
```bash
vagrant halt -f && vagrant destroy -f
```
