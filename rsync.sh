ROBOT_IP=192.168.32.198
sshpass -p "modpow" rsync -rhav --delete ~/tfm/tfm_ws/devel/include/ everis@$ROBOT_IP:/opt/rosslam_ws/devel/include/
sshpass -p "modpow" rsync -rhav --delete ~/tfm/tfm_ws/devel/lib/ everis@$ROBOT_IP:/opt/rosslam_ws/devel/lib/
sshpass -p "modpow" rsync -rhav --delete ~/tfm/tfm_ws/devel/share/ everis@$ROBOT_IP:/opt/rosslam_ws/devel/share/
sshpass -p "modpow" rsync -rhav --delete --exclude={'src','include','.git','CMakeLists.txt','ouster_neo_test.launch'} ~/tfm/tfm_ws/src/ everis@$ROBOT_IP:/opt/rosslam_ws/src/
