import sys
import rclpy
from rclpy.node import Node
import xacro
from ament_index_python.packages import get_package_share_path


class GenSdf(Node):
    def __init__(self):
        super().__init__("gen_sdf")

        self.declare_parameter("name", "unknown")
        self.name = self.get_parameter('name').get_parameter_value().string_value

        self.declare_parameter("type", "unknown")
        self.type = self.get_parameter('type').get_parameter_value().string_value

        self.declare_parameter("xacro_file", "")
        self.xacro_file = self.get_parameter('xacro_file').get_parameter_value().string_value

        self.declare_parameter("with_camera", False)
        self.with_camera = self.get_parameter('with_camera').get_parameter_value().bool_value

        self.declare_parameter("camera_name", "camera0")
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        self.declare_parameter("laser_name", "laser0")
        self.laser_name = self.get_parameter('laser_name').get_parameter_value().string_value

        self.declare_parameter("robot_name", "unknown")
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.declare_parameter("robot", False)
        self.robot = self.get_parameter('robot').get_parameter_value().bool_value

        self.declare_parameter("gimbal", False)
        self.gimbal = self.get_parameter('gimbal').get_parameter_value().bool_value

        self.declare_parameter("laser", False)
        self.laser = self.get_parameter('laser').get_parameter_value().bool_value

        self.declare_parameter("camera_update_rate", 30)
        self.camera_update_rate = self.get_parameter('camera_update_rate').get_parameter_value().integer_value
        
        self.declare_parameter("laser_update_rate", 10)
        self.laser_update_rate = self.get_parameter('laser_update_rate').get_parameter_value().integer_value
        
        self.robot_gimbal = True

        #self.get_logger().error(f'GenSdf: {self.name}')        
        #self.get_logger().error(f'GenSdf: {self.type}')        
        #self.get_logger().error(f'GenSdf: {self.xacro_file}')
        #self.get_logger().error(f'GenSdf: {self.with_camera}')


    def process(self):
        res = "dummmy"
        if self.gimbal or self.laser:
            self.robot_gimbal = False
        elif self.robot and not self.with_camera:
            self.robot_gimbal = False
        if not self.xacro_file:
            xacro_path = get_package_share_path('lrs_halmstad') / 'xacro'
            if self.robot_gimbal:
                self.xacro_file = f'{xacro_path}/lrs_model.xacro'
            elif self.robot:
                self.xacro_file = f'{xacro_path}/lrs_robot.xacro'
            elif self.gimbal:
                self.xacro_file = f'{xacro_path}/lrs_gimbal.xacro'
            elif self.laser:
                self.xacro_file = f'{xacro_path}/lrs_laser.xacro'
        mappings = {}
        mappings["robot_type"] = self.type

        if self.robot_gimbal:
            mappings["robot_name"] = self.name
            mappings["camera_name"] = self.camera_name
            if self.with_camera:
                mappings["with_camera"] = "true"
            else:
                mappings["with_camera"] = "false"
                
        if self.robot:
            mappings["name"] = self.name

        if self.gimbal:
            mappings["camera_name"] = self.camera_name
            mappings["robot_name"] = self.robot_name
            mappings["camera_far_clip"] = "900"
            mappings["camera_update_rate"] = f'{self.camera_update_rate}'

        if self.laser:
            mappings["laser_name"] = self.laser_name
            mappings["robot_name"] = self.robot_name
            mappings["laser_far_clip"] = "900"
            mappings["laser_update_rate"] = f'{self.laser_update_rate}'

            
        # self.get_logger().error(f'GenSdf XACROFILE: {self.xacro_file}')                        
        # self.get_logger().error(f'GenSdf MAPPINGS: {mappings}')
        try:
            doc = xacro.process_file(self.xacro_file, mappings=mappings)
        except Exception as ex:
            print("ACROFILE:", self.xacro_file, mappings)
            print("Exception:", ex, type(ex))
            sys.exit(1)
        
        res = doc.toprettyxml(indent='  ')
        # self.get_logger().error(f'SDF: {res}')                                
        return res

def main(args=None):
    rclpy.init(args=args)
    node = GenSdf()
    res = node.process()

    #f = open("/tmp/gen.sdf", "w")
    #f.write(res)
    #f.close()
    
    # node.get_logger().error(f'GenSdf RESULT: {res}')
    print(res)

    # return "/tmp/gen.sdf"


    
