#!/usr/bin/env bash

vagrant halt -f && vagrant destroy -f && vagrant up && vagrant ssh
