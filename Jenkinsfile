pipeline {
    agent any
    triggers {
        pollSCM('H/5 * * * *')
    }
    options {
        checkoutToSubdirectory('src/franka_ros')
    }
    stages {
        stage('Run on noetic') {
             agent {
                 dockerfile {
                     filename '.ci/Dockerfile.noetic'
                     dir 'src/franka_ros'
                 }
             }
            steps {
                script {
                    notifyBitbucket()
                }
                sh ''' . /opt/ros/noetic/setup.sh
                    export HOME=$(pwd)
                    ./src/franka_ros/.ci/debug.sh
                '''
            }
            post {
                always {
                    junit 'build-debug/test_results/**/*.xml'
                }
            }
        }
        stage('Run on melodic') {
            agent{
                dockerfile {
                     filename '.ci/Dockerfile.melodic'
                      dir 'src/franka_ros'
                 }
            }
            steps {
                 sh ''' . /opt/ros/melodic/setup.sh
                     export HOME=$(pwd)
                     ./src/franka_ros/.ci/debug.sh
                 '''
            }
            post {
                always {
                    junit 'build-debug/test_results/**/*.xml'
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
