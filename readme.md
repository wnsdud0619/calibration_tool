
요구사항 ubuntu 18.04, opnecv 3.4.17, ros-melodic

autoware_camera_lidar_calibration   
extract_RT   
calibration_check   
순서대로 파일 실행하여 calibration 진행.   


# opencv 3.4.17설치 방법.       
------------
기존에 설치된 항목 지우기   
sudo apt-get remove libopencv*   
sudo apt-get autoremove   

업데이트   
sudo apt-get update   
sudo apt-get upgrade   

필수개발툴 설치   

	sudo apt-get install build-essential cmake unzip pkg-config   

이미지처리와 비전 라이브러리에 관한 패키지설치를 진행합니다.   

	sudo apt-get install libjpeg-dev libpng-dev libtiff-dev   

카메라 스트림으로 비디오 파일을 처리할 수있는 패키지를 설치   

	sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev v4l-utils libxvidcore-dev libx264-dev libxine2-dev   

비디오 스트리밍관련 패키지설치   

	sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev   

opencv내부의 GUI 작업을 위한 GTX에 관한 라이브러리설치   

	sudo apt-get install libgtk-3-dev   

openGL의 사용을 위한 라이브러리설치   

	sudo apt-get install mesa-utils libgl1-mesa-dri libgtkgl2.0-dev libgtkglext1-dev   

기능 최적화를 위한 라이브러리설치   

	sudo apt-get install libatlas-base-dev gfortran libeigen3-dev   

python관련 설치   

	sudo apt-get install python2.7-dev python3-dev python-numpy python3-numpy   

opencv 다운받아 build   

	mkdir opencv   
	cd opencv   
	wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.17.zip   
	wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.17.zip   
	unzip opencv.zip   
	unzip opencv_contrib.zip   
	cd opencv-3.4.17   

	mkdir build   
	cd build   
	cmake -D CMAKE_BUILD_TYPE=RELEASE\
	-D CMAKE_INSTALL_PREFIX=/usr/local\
	-D WITH_TBB=OFF\
	-D WITH_IPP=OFF\
	-D WITH_1394=OFF\
	-D WITH_GTK_2_X=ON\
	-D BUILD_WITH_DEBUG_INFO=OFF\
	-D BUILD_DOCS=OFF\
	-D INSTALL_C_EXAMPLES=ON\
	-D INSTALL_PYTHON_EXAMPLES=ON\
	-D BUILD_EXAMPLES=OFF\
	-D BUILD_TESTS=OFF\
	-D BUILD_PERF_TESTS=OFF\
	-D WITH_QT=OFF\
	-D WITH_GTK=ON\
	-D WITH_OPENGL=ON\
	-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-3.4.17/modules\
	-D WITH_V4L=ON\
	-D WITH_FFMPEG=ON\
	-D WITH_XINE=ON\
	-D BUILD_NEW_PYTHON_SUPPORT=ON\
	-D PYTHON2_INCLUDE_DIR=/usr/include/python2.7\
	-D PYTHON2_NUMPY_INCLUDE_DIRS=/usr/lib/python2.7/dist-packages/numpy/core/include/\
	-D PYTHON2_PACKAGES_PATH=/usr/lib/python2.7/dist-packages\
	-D PYTHON2_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython2.7.so\
	-D PYTHON3_INCLUDE_DIR=/usr/include/python3.6m\
	-D PYTHON3_NUMPY_INCLUDE_DIRS=/usr/lib/python3/dist-packages/numpy/core/include/\
	-D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages\
	-D PYTHON3_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so\
	../   

본인 컴퓨터 CPU 코어수 확인하여 make진행   

	make -j12   
	sudo make install   
	sudo sh -c echo '/usr/local/lib/' > sudo /etc/ld.so.conf.d/opencv.conf   
	sudo ldconfig   






