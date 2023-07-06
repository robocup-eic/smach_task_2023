
from cv_connector.msg import CV_type
from cv_connector.srv import CV_srv, CV_srvRequest, CV_srvResponse



cv_client = rospy.ServiceProxy('/CV_connect/req_cv', CV_srv)
req = CV_srvRequest()
req.cv_type.type = CV_type.HumanPoseEstimation