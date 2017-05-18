CFLAGS =  -DCOMPILING_EXTERNAL_COLL_DIST_DLL -Wall -fPIC -static

OS = $(shell uname -s)
ifeq ($(OS), Linux)
    CFLAGS += -D__linux -DLIN_VREP
    EXT = so
else
    CFLAGS += -D__APPLE__ -DMAC_VREP
    EXT = dylib
endif

CFLAGS += -I../programming/include
CFLAGS += -I../programming/v_repMath
CFLAGS += -IsourceCode
CFLAGS += -IsourceCode/collDistAlgos

all: 
    @rm -f lib/*.*
    @rm -f *.o 

    gcc $(CFLAGS) -c sourceCode/collDistAlgos/collDistAlgos.cpp -o collDistAlgos.o
    gcc $(CFLAGS) -c sourceCode/collDistAlgos/collNode.cpp -o collNode.o
    gcc $(CFLAGS) -c sourceCode/collDistAlgos/collInfo.cpp -o collInfo.o
    gcc $(CFLAGS) -c sourceCode/collDistAlgos/pointCloudNode.cpp -o pointCloudNode.o
    gcc $(CFLAGS) -c sourceCode/collDistAlgos/pointCloudInfo.cpp -o pointCloudInfo.o
    gcc $(CFLAGS) -c sourceCode/collDistAlgos/octreeNode.cpp -o octreeNode.o
    gcc $(CFLAGS) -c sourceCode/collDistAlgos/octreeInfo.cpp -o octreeInfo.o
    gcc $(CFLAGS) -c sourceCode/collDistAlgos/kdTreeNode.cpp -o kdTreeNode.o
    gcc $(CFLAGS) -c sourceCode/collDistAlgos/kdTreeInfo.cpp -o kdTreeInfo.o
    gcc $(CFLAGS) -c sourceCode/collDistAlgos/linkedListElement.cpp -o linkedListElement.o
    gcc $(CFLAGS) -c sourceCode/v_repExtMeshCalc.cpp -o v_repExtMeshCalc.o
    gcc $(CFLAGS) -c ../programming/common/v_repLib.cpp -o v_repLib.o
    gcc $(CFLAGS) -c ../programming/v_repMath/3Vector.cpp -o 3Vector.o
    gcc $(CFLAGS) -c ../programming/v_repMath/4Vector.cpp -o 4Vector.o
    gcc $(CFLAGS) -c ../programming/v_repMath/7Vector.cpp -o 7Vector.o
    gcc $(CFLAGS) -c ../programming/v_repMath/3X3Matrix.cpp -o 3X3Matrix.o
    gcc $(CFLAGS) -c ../programming/v_repMath/4X4Matrix.cpp -o 4X4Matrix.o
    gcc $(CFLAGS) -c ../programming/v_repMath/MyMath.cpp -o MyMath.o

    @mkdir -p lib
    gcc *.o -o lib/libv_repExtMeshCalc.$(EXT) -lpthread -ldl -shared
