pipeline {
    agent any 
    options { 
        disableConcurrentBuilds() 
        ansiColor('xterm')
        lock resource: 'docker'
        timeout(time:30 , unit: 'MINUTES')
    }
    environment {
        ADORE_DOCKER_WORKSPACE = "/var/jenkins_home/jobs/docker/jobs/docker/workspace"
        OUTPUT_DIRECTORY_BASE = "/var/log/adore/out"
        DOCKER_CONFIG="tools"
    }
    stages {
        stage('Build') {
            steps {
                sh '''#!/usr/bin/env bash
                    set -e
                    export DOCKER_CONFIG=$(realpath "${DOCKER_CONFIG}") make
                    make
                '''
            }
        }
        stage('Test') {
            steps {
                sh '''#!/usr/bin/env bash
                    set -e
                    export DOCKER_CONFIG=$(realpath "${DOCKER_CONFIG}") make
                    make test
                 '''
            }
        }
        stage('Static Checks: Lint') {
            steps {
                sh '''#!/usr/bin/env bash
                    set -e
                    export DOCKER_CONFIG=$(realpath "${DOCKER_CONFIG}") make
                    make lint
                 '''
            }
        }
        stage('Static Checks: Static Code Checking') {
            steps {
                sh '''#!/usr/bin/env bash
                    set -e
                    export DOCKER_CONFIG=$(realpath "${DOCKER_CONFIG}") make
                    make cppcheck
                 '''
            }
        }
        stage('Static Checks: Lizard Report') {
            steps {
                sh '''#!/usr/bin/env bash
                    set -e
                    export DOCKER_CONFIG=$(realpath "${DOCKER_CONFIG}") make
                    make lizard
                 '''
            }
        }
    }
}

