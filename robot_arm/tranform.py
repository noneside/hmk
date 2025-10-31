# convert_urdf.py
import mujoco
import os 

def convert_urdf_to_mujoco_xml(urdf_path, output_xml_path):
    """
    将 URDF 文件转换为 MuJoCo MJCF XML 文件。

    Args:
        urdf_path (str): 输入的 URDF 文件路径。
        output_xml_path (str): 输出的 MJCF XML 文件路径。
    """
    try:
        # 1. 从 URDF 文件加载模型。MuJoCo 的编译器会自动处理转换。
        # 这一步会在内存中创建一个 mujoco.MjModel 实例。
        print(f"正在加载 URDF 文件: {urdf_path}")
        model = mujoco.MjModel.from_xml_path(urdf_path)
        print("URDF 文件加载并编译成功。")

        # 2. 将最后编译的模型保存为 XML 字符串。
        # 这个函数会获取上一次编译操作（即 from_xml_path）的结果。
        # 如果模型无法编译，此函数将引发错误。
        xml_string = mujoco.mj_saveLastXML(output_xml_path, model)
        
        # mj_saveLastXML 已经将文件写入磁盘，这里只是为了确认
        if os.path.exists(output_xml_path):
            print(f"成功将模型保存为 MuJoCo XML 文件: {output_xml_path}")
        else:
            # 如果 mj_saveLastXML 由于某种原因失败（例如权限问题），则会进入这里
            raise IOError("mj_saveLastXML 未能创建文件。")

    except Exception as e:
        print(f"转换过程中发生错误: {e}")

if __name__ == '__main__':
    # 确保 simple_arm.urdf 文件和此脚本在同一目录下
    urdf_file = '/home/hmk/robot_arm/urdf/RM65-6F.urdf'
    xml_file = '/home/hmk/robot_arm/urdf/65.xml'
    convert_urdf_to_mujoco_xml(urdf_file, xml_file)


