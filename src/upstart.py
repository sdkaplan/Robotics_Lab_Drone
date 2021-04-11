import robot_upstart
st=robot_upstart.Job()
st.add(package="Robotics_Lab_UAV",filename="Data_collection_UAV.launch")
st.install()
