#!groovy

node {
  step([$class: 'StashNotifier'])

  def gitCommit
  try {
    stage('Checkout') {
      dir('catkin_ws/src') {
        checkout scm
        gitCommit = sh(script: 'git rev-parse HEAD', returnStdout: true).trim()
      }

      dir('libfranka') {
        // TODO(walc_fl): Remove hard-coded repository URL
        checkout resolveScm(source: [$class: 'GitSCMSource', remote: 'https://github.com/frankaemika/libfranka.git',
                                     credentialsId: '6a639baf-566a-4e66-b089-74cd9ecb38a8', includes: '*', excludes: '',
                                     extensions: [[$class: 'SubmoduleOption', parentCredentials: true, recursiveSubmodules: true]]],
                            targets: [BRANCH_NAME, 'master'])
      }
    }

    stage('Build libfranka') {
      dir('libfranka/build') {
        sh 'cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_COVERAGE=OFF -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCUMENTATION=OFF ..'
        sh 'cmake --build .'
      }
    }

    stage('Build in debug mode') {
      dir('catkin_ws') {
        env.FRANKA_DIR = "${pwd()}/../libfranka/build"
        sh 'src/scripts/ci/debug-build.sh'
      }
    }

    stage('Archive results') {
      junit 'build/test_results/**/*.xml'
    }
    currentBuild.result = 'SUCCESS'
  } catch (e) {
    currentBuild.result = 'FAILED'
    throw e;
  } finally {
    // Explicitly specify commit hash, otherwise Stash will get notified about the `libfranka` commit as well.
    if (gitCommit) {
      step([$class: 'StashNotifier', commitSha1: gitCommit])
    } else {
      step([$class: 'StashNotifier'])
    }
  }
}
