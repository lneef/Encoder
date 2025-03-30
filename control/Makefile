SRCS = $(wildcard src/*.cc)
CXXFLAGS = -std=c++20 -Wall -Wpedantic -O3 
LDFLAGS = -I include/ -lboost_thread -lboost_locale -lboost_system -lboost_chrono
OBJS = $(SRCS:.cc=.o)
TARGET = main

APP = test_$(TARGET)
APP_SRC = test/test.cc
APP_OBJ = $(APP_SRC:.cc=.o)

all: $(TARGET)

test: $(APP_OBJ)
	$(CXX) $(CXXFLAGS) -o $(APP) $(APP_OBJ) $(LDFLAGS)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@ $(LDFLAGS)

clean:
	rm -f $(TARGET) $(OBJS)
