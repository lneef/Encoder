SRCS = $(wildcard src/*.cc)
CXXFLAGS = -std=c++20 -Wno-interference-size -Wall -O3 
LDFLAGS = -I include/ -lboost_thread -lboost_locale -lboost_system -lboost_chrono
OBJS = $(SRCS:.cc=.o)
TARGET = main

APP = example/main
APP_SRC = example/test.cc
APP_OBJ = $(APP_SRC:.cc=.o)

all: $(TARGET) 

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGET) $(OBJS)
