pipeline {
    agent any
    triggers {
        pollSCM('H/5 * * * *')
    }
    libraries {
        lib('fe-pipeline-steps@1.5.0')
    }
    options {
        checkoutToSubdirectory('src/franka_ros')
        parallelsAlwaysFailFast()
        disableConcurrentBuilds()
    }
    environment {
        CMAKE_BUILD_PARALLEL_LEVEL=sh(script: 'nproc', returnStdout: true).trim().toInteger()
    }
    stages {
        stage('Build & Test') {
            matrix {
                agent {
                    dockerfile {
                        filename ".ci/Dockerfile.${env.DISTRO}"
                        dir 'src/franka_ros'
                    }
                }
                axes {
                    axis {
                        name 'DISTRO'
                        values 'melodic', 'noetic'
                    }
                    axis {
                        name 'BUILD_TOOL'
                        values 'catkin_make', 'catkin build'
                    }
                }
                stages {
                    stage('Notify Stash') {
                        when {
                            allOf {
                                environment name: 'DISTRO', value: 'noetic'
                                environment name: 'BUILD_TOOL', value: 'catkin_make'
                            }
                        }
                        steps {
                            script {
                                notifyBitbucket()
                            }
                        }
                    }
                    stage('Build w/ Catkin Make') {
                        when {
                            environment name: 'BUILD_TOOL', value: 'catkin_make'
                        }
                        steps {
                            sh '''
                                . /opt/ros/${DISTRO}/setup.sh
                                rm -rf src/CMakeLists.txt build devel
                                catkin_init_workspace src
                                ${BUILD_TOOL} -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
                            '''
                        }
                    }
                    stage('Build w/ Catkin Tools') {
                        when {
                            environment name: 'BUILD_TOOL', value: 'catkin build'
                        }
                        environment {
                            HOME=sh(script: 'pwd', returnStdout: true).trim()
                            SPACE_SUFFIX='_catkin_tools'
                        }
                        steps {
                            sh '''
                                . /opt/ros/${DISTRO}/setup.sh
                                rm -rf *${SPACE_SUFFIX}
                                catkin config --workspace . --init --extend /opt/ros/${DISTRO} \\
                                    --space-suffix ${SPACE_SUFFIX}
                                ${BUILD_TOOL} --no-status
                            '''
                        }
                    }
                    stage('Check C++ Format') {
                        when {
                            environment name: 'BUILD_TOOL', value: 'catkin_make'
                        }
                        steps {
                            sh '''
                                cmake --build build --target check-format
                            '''
                        }
                    }
                    stage('Check Python Format') {
                        when {
                            environment name: 'BUILD_TOOL', value: 'catkin_make'
                        }
                        steps {
                            sh '''
                                cmake --build build --target check-pyformat
                            '''
                        }
                    }
                    stage('Check Linting') {
                        when {
                            environment name: 'BUILD_TOOL', value: 'catkin_make'
                        }
                        steps {
                            sh '''
                                . /opt/ros/${DISTRO}/setup.sh
                                cmake --build build --target check-tidy
                            '''
                        }
                    }
                    stage('Test') {
                        when {
                            environment name: 'BUILD_TOOL', value: 'catkin_make'
                        }
                        environment {
                            HOME=sh(script: 'pwd', returnStdout: true).trim()
                        }
                        steps {
                            sh '''
                                . /opt/ros/${DISTRO}/setup.sh
                                ${BUILD_TOOL} run_tests -j1
                                catkin_test_results
                            '''
                        }
                        post {
                            always {
                                junit 'build/test_results/**/*.xml'
                            }
                        }
                    }
                    stage('Check for Non-ASCII') {
                        when {
                            // Melodic has problems with non-ascii chars in YAML files
                            environment name: 'DISTRO', value: 'melodic'
                        }
                        steps {
                            dir('src/franka_ros') {
                                feEnsureAsciiFileContents('*.yaml')
                            }
                        }
                    }
                    stage('Check commit history sync') {
                        when {
                            allOf {
                                environment name: 'DISTRO', value: 'noetic'
                                environment name: 'BUILD_TOOL', value: 'catkin_make'
                            }
                        }
                        steps {
                            sh """
                                cd src/franka_ros
                                .ci/checkgithistory.sh \\
                                    https://github.com/frankaemika/franka_ros.git develop
                            """
                        }
                    }
                }
            }
        }
    }
    post {
        always {
            cleanWs()
            script {
                notifyBitbucket()
            }
        }
    }
}
