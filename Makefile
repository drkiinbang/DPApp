# DPApp Makefile

# 컴파일러 설정
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Iinclude -Isrc

# 디렉토리 설정
SRCDIR = src
INCDIR = include
BINDIR = bin
OBJDIR = obj

# 플랫폼별 설정
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
    LIBS = -lpthread
endif
ifeq ($(UNAME_S),Darwin)
    LIBS = -lpthread
endif
ifeq ($(OS),Windows_NT)
    LIBS = -lws2_32 -lwsock32
    CXX = x86_64-w64-mingw32-g++
endif

# 소스 파일들
COMMON_SOURCES = $(wildcard $(SRCDIR)/common/*.cpp) $(wildcard $(SRCDIR)/network/*.cpp) $(wildcard $(SRCDIR)/pointcloud/*.cpp)
MASTER_SOURCES = $(wildcard $(SRCDIR)/master/*.cpp)
SLAVE_SOURCES = $(wildcard $(SRCDIR)/slave/*.cpp)

# 오브젝트 파일들
COMMON_OBJECTS = $(COMMON_SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
MASTER_OBJECTS = $(MASTER_SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
SLAVE_OBJECTS = $(SLAVE_SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)

# 실행 파일들
MASTER_TARGET = $(BINDIR)/master
SLAVE_TARGET = $(BINDIR)/slave

# 기본 타겟
all: directories $(MASTER_TARGET) $(SLAVE_TARGET)

# 디렉토리 생성
directories:
	@mkdir -p $(OBJDIR)/common $(OBJDIR)/network $(OBJDIR)/pointcloud $(OBJDIR)/master $(OBJDIR)/slave
	@mkdir -p $(BINDIR)

# Master 실행 파일
$(MASTER_TARGET): $(COMMON_OBJECTS) $(MASTER_OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)
	@echo "Master executable created: $@"

# Slave 실행 파일
$(SLAVE_TARGET): $(COMMON_OBJECTS) $(SLAVE_OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)
	@echo "Slave executable created: $@"

# 오브젝트 파일 컴파일
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 개별 타겟들
master: directories $(MASTER_TARGET)

slave: directories $(SLAVE_TARGET)

# 테스트 실행
test: all
	@echo "Running sample test..."
	@echo "Start master in one terminal: $(MASTER_TARGET)"
	@echo "Start slave in another terminal: $(SLAVE_TARGET)"
	@echo "In master console, type: load data/sample_points.xyz filter"

# 정리
clean:
	rm -rf $(OBJDIR) $(BINDIR)
	@echo "Cleaned build files"

# 전체 재빌드
rebuild: clean all

# 설치 (Linux/Mac용)
install: all
	@mkdir -p /usr/local/bin
	@cp $(MASTER_TARGET) /usr/local/bin/
	@cp $(SLAVE_TARGET) /usr/local/bin/
	@echo "Installed to /usr/local/bin/"

# 도움말
help:
	@echo "DPApp Build System"
	@echo "=================="
	@echo "Targets:"
	@echo "  all       - Build both master and slave"
	@echo "  master    - Build master only"
	@echo "  slave     - Build slave only"
	@echo "  test      - Show test instructions"
	@echo "  clean     - Remove build files"
	@echo "  rebuild   - Clean and build all"
	@echo "  install   - Install to system (Linux/Mac)"
	@echo "  help      - Show this help"

.PHONY: all directories master slave test clean rebuild install help