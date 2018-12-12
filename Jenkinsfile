#!/usr/bin/env groovy

node('delphyne_linux_bionic_unprovisioned') {
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
