TARGET ?= ./bin/a.exe

CFLAGS = -MMD -MP
LDFLAGS = 

SRC_DIR ?= ./src
SRCS := $(shell find $(SRC_DIR) -name *.cpp)

OBJ_DIR ?= ./obj
OBJS := $(addprefix $(OBJ_DIR)/,$(notdir $(SRCS:.cpp=.o)))
DEPS := $(OBJS:.o=.d)

INC_DIR ?= ./inc
INC_FLAGS := -I$(INC_DIR)


$(TARGET): $(OBJS)
	g++ -o $@ $^ $(LDFLAGS)


$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	mkdir -p $(OBJ_DIR)
	g++ $(CFLAGS) $(INC_FLAGS) -o $@ -c $<

all: clean $(TARGET)

clean:
	-rm -f $(OBJS) $(DEPS) $(TARGET)

-include $(DEPS)