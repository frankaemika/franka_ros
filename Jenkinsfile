pipeline {
    agent any
    triggers {
        pollSCM('H/5 * * * *')
    }
    options {
        checkoutToSubdirectory('src/franka_ros')
    }
    stages {
        stage('Notify Stash') {
            steps {
                script {
                    notifyBitbucket()
                }
            }
        }
        stage('Build & Test') {
            parallel {
                stage('Melodic') {
                    agent {
                        dockerfile {
                            filename '.ci/Dockerfile.melodic'
                            dir 'src/franka_ros'
                        }
                    }
                    stages {
                        stage('Build') {
                            steps {
                                sh ''' . /opt/ros/melodic/setup.sh
                                    ./src/franka_ros/.ci/build.sh
                                '''
                            }
                        }
                        stage('Test') {
                            steps {
                                sh ''' . /opt/ros/melodic/setup.sh
                                    export HOME=$(pwd)
                                    ./src/franka_ros/.ci/test.sh
                                '''
                            }
                            post {
                                always {
                                    junit 'build-debug/test_results/**/*.xml'
                                }
                            }
                        }
                    }
                }
                stage('Noetic') {
                    agent {
                        dockerfile {
                            filename '.ci/Dockerfile.noetic'
                            dir 'src/franka_ros'
                        }
                    }
                    stages {
                        stage('Build') {
                            steps {
                                sh ''' . /opt/ros/noetic/setup.sh
                                    ./src/franka_ros/.ci/build.sh
                                '''
                            }
                        }
                        stage('Test') {
                            steps {
                                sh ''' . /opt/ros/noetic/setup.sh
                                    export HOME=$(pwd)
                                    ./src/franka_ros/.ci/test.sh
                                '''
                            }
                            post {
                                always {
                                    junit 'build-debug/test_results/**/*.xml'
                                }
                            }
                        }
                    }
                }
            }
        }
        stage('Check commit history sync') {
            agent {
                dockerfile {
                    filename '.ci/Dockerfile.noetic'
                    dir 'src/franka_ros'
                }
            }
            steps {
                sh """
                cd src/franka_ros
                .ci/checkgithistory.sh https://github.com/frankaemika/franka_ros.git develop
                """
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
