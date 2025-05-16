FROM lukstep/raspberry-pi-pico-sdk:latest AS sdk_setup

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y \
    wget \
    python3-numpy \
    zlib1g-dev \
    pkg-config \
    vim

# libpng installation
# ARG LIBPNG_PATH=/usr/local/libpng
# RUN mkdir $LIBPNG_PATH
# RUN wget https://download.sourceforge.net/libpng/libpng-1.6.44.tar.xz
# RUN tar xf libpng-1.6.44.tar.xz -C $LIBPNG_PATH
# RUN cd $LIBPNG_PATH/libpng-1.6.44 && \
#     # ls && \
#     ./configure && \
#     make check && \
#     make install

# libfftw3 installation
# ARG LIBFFTW3_PATH=/usr/local/libfftw3
# RUN mkdir $LIBFFTW3_PATH
# RUN wget https://www.fftw.org/fftw-3.3.10.tar.gz
# RUN tar xf fftw-3.3.10.tar.gz -C $LIBFFTW3_PATH 
# RUN cd $LIBFFTW3_PATH/fftw-3.3.10 && \
#     ./configure && \
#     make && \
#     make install

# kissfft installation
# ARG KISSFFT_PATH=/usr/local/kissfft
# RUN git clone --depth 1 https://github.com/mborgerding/kissfft.git $KISSFFT_PATH && \
#     cd $KISSFFT_PATH && \
#     mkdir build && cd build && \
#     cmake -DKISSFFT_DATATYPE=float -DKISSFFT_STATIC=ON -DKISSFFT_OPENMP=ON .. && \
#     make all && make install


