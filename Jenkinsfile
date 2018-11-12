#!groovy

def get_stages(ubuntu_version, ros_distribution){
  node('docker') {
  step([$class: 'StashNotifier'])

  try {
    dir('src/franka_ros') {
      checkout scm
    }

    sh 'rm -rf dist'
    dir('dist') {
      try {
        step([$class: 'CopyArtifact',
              filter: 'libfranka-*-amd64.tar.gz',
              fingerprintArtifacts: true,
              projectName: "SWDEV/libfranka/${java.net.URLEncoder.encode(env.BRANCH_NAME, "UTF-8")}",
              selector: [$class: 'StatusBuildSelector', stable: false]])
      } catch (e) {
        // Fall back to develop branch.
        step([$class: 'CopyArtifact',
              filter: 'libfranka-*-amd64.tar.gz',
              fingerprintArtifacts: true,
              projectName: "SWDEV/libfranka/develop",
              selector: [$class: 'StatusBuildSelector', stable: false]])
      }
      sh '''
        tar xfz libfranka-*-amd64.tar.gz
        ln -sf libfranka-*-amd64 libfranka
      '''
    }

    docker.build("franka_ros-ci-worker:${ubuntu_version}", "-f src/franka_ros/.ci/Dockerfile.${ubuntu_version} .ci").inside {
      withEnv(["CMAKE_PREFIX_PATH+=${env.WORKSPACE}/dist/libfranka/lib/cmake/Franka",
               "ROS_HOME=${env.WORKSPACE}/ros-home"]) {
        stage('Build & Lint (Debug)') {
          sh ". /opt/ros/${ros_distribution}/setup.sh"
          sh 'src/franka_ros/.ci/debug.sh'
          junit 'build-debug/test_results/**/*.xml'
        }
      }
    }

    currentBuild.result = 'SUCCESS'
  } catch (e) {
    currentBuild.result = 'FAILED'
    throw e;
  } finally {
    step([$class: 'StashNotifier'])
  }
}

}


parallel(
  'xenial': get_stages('xenial, kinetic'),
  'bionic': get_stages('bionic, melodic'),
)
