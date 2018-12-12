#!/usr/bin/env groovy

node('delphyne-linux-bionic-unprovisioned') {
  // From empirical evidence it takes ~10 minutes to install dependencies
  // and ~20 minutes to build and run the tests.  That adds up to 30 minutes
  // which we double to 60 to give us enough leeway.
  timeout(60) {
    ansiColor('xterm') {
      try {
        stage('checkout') {
          checkout scm
        }
        stage('setup early') {
          sh './tools/continuous_integration/jenkins/setup_early'
        }
        stage('check style') {
          sh './tools/continuous_integration/jenkins/check_style'
        }
        stage('build workspace') {
          sh './tools/continuous_integration/jenkins/build_workspace'
        }
        stage('setup late') {
          sh './tools/continuous_integration/jenkins/setup_late'
        }
        stage('build') {
          sh './tools/continuous_integration/jenkins/build'
        }
        stage('test') {
          sh './tools/continuous_integration/jenkins/test'
        }
      } finally {
        cleanWs(notFailBuild: true)
      }
    }
  }
}
