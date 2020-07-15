#!groovy

buildResult = 'NOT_BUILT'

def getStages(rosDistribution, ubuntuVersion) {
  return {
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
                  filter: "libfranka-*-amd64-${ubuntuVersion}.tar.gz",
                  fingerprintArtifacts: true,
                  projectName: "SWDEV/libfranka/${java.net.URLEncoder.encode(env.BRANCH_NAME, "UTF-8")}",
                  selector: [$class: 'StatusBuildSelector', stable: false]])
          } catch (e) {
            // Fall back to develop branch.
            step([$class: 'CopyArtifact',
                  filter: "libfranka-*-amd64-${ubuntuVersion}.tar.gz",
                  fingerprintArtifacts: true,
                  projectName: "SWDEV/libfranka/develop",
                  selector: [$class: 'StatusBuildSelector', stable: false]])
          }
          sh """
            tar xfz libfranka-*-amd64-${ubuntuVersion}.tar.gz
            ln -sf libfranka-*-amd64 libfranka
          """
        }

        docker.build("franka_ros-ci-worker:${rosDistribution}",
                     "-f src/franka_ros/.ci/Dockerfile.${rosDistribution} src/franka_ros/.ci").inside('-e MAKEFLAGS') {
          withEnv(["CMAKE_PREFIX_PATH+=${env.WORKSPACE}/dist/libfranka/lib/cmake/Franka",
                   "ROS_HOME=${env.WORKSPACE}/ros-home"]) {
            stage("${rosDistribution}: Build & Lint (Debug)") {
              sh """
                . /opt/ros/${rosDistribution}/setup.sh
                src/franka_ros/.ci/debug.sh
              """
              junit 'build-debug/test_results/**/*.xml'
            }
          }
        }

        if (buildResult != 'FAILED') {
          buildResult = 'SUCCESS'
        }
      } catch (e) {
        buildResult = 'FAILED'
      }
    }
  }
}

node {
  step([$class: 'StashNotifier'])
}

parallel(
  'kinetic': getStages('kinetic', 'xenial'),
  'melodic': getStages('melodic', 'bionic'),
  'noetic': getStages('noetic', 'focal'),
)

node {
  currentBuild.result = buildResult
  step([$class: 'StashNotifier'])
}
