## interface接口：
参考https://roboticsbackend.com/ros2-create-custom-message/

【1】，在工作区的代码区中创建我们的接口文件包：
![[Pasted image 20250922211155.png]]

【2】，进入我们创建的这个文件夹中，然后删掉include和src这两个文件夹，只留CMakeList和package
![[Pasted image 20250922211341.png]]
![[Pasted image 20250922211351.png]]

【3】，在package这个文件里面编辑，在中间部分加入这三行：
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
![[Pasted image 20250922211701.png]]

【4】，在Cmake中编辑，把多余内容删掉，加入这部分：
![[Pasted image 20250922211845.png]]
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
【这部分写你自己定义的节点】
)
ament_export_dependencies(rosidl_default_runtime)
![[Pasted image 20250922211959.png]]

【5】，在这个接口包里面创建一个文件夹叫msg，在这个文件夹里面创建接口的具体文件.msg后缀
![[Pasted image 20250922212238.png]]
（注意这个接口文件.msg，命名必须用驼峰命名法，不能用下划线）

【6】，在这个接口文件里面写具体的接口格式
![[Pasted image 20250922212339.png]]
然后在Cmake文件里面写入我们自定义的接口路径（在接口文件夹下的路径）
![[Pasted image 20250922212619.png]]


【7】，然后重新构建一下工作区
![[Pasted image 20250922212633.png]]

【8】，然后可以测试查看一下我们定义的接口
![[Pasted image 20250922212721.png]]
