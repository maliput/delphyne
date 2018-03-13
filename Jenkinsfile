#!/usr/bin/env groovy

node('linux_xenial_unprovisioned') {
  try {
    stage('checkout') {
      checkout scm
    }
    stage('setup early') {
      sh './scripts/continuous_integration/jenkins/setup_early'
    }
    stage('check style') {
      sh './scripts/continuous_integration/jenkins/check_style'
    }
    stage('build workspace') {
      sh './scripts/continuous_integration/jenkins/build_workspace'
    }
    stage('setup late') {
      sh './scripts/continuous_integration/jenkins/setup_late'
    }
    stage('build') {
      sh './scripts/continuous_integration/jenkins/build'
    }
    stage('test') {
      sh './scripts/continuous_integration/jenkins/test'
    }
  } finally {
    cleanWs(notFailBuild: true)
  }
}
