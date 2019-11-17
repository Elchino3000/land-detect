g++ main.cpp -o camera -I/opt/ros/indigo/include -L/opt/ros/indigo/lib -Wl,-rpath,/opt/ros/hydro/lib -lroscpp -lrosconsole -lrostime  `pkg-config --cflags --libs opencv image_transport cv_bridge`

