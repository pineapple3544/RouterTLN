FROM misys:forza_full
USER root
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-toolkit-12-8 g++-11 gcc-11 ninja-build curl unzip \
    libyaml-cpp-dev nlohmann-json3-dev libpng-dev python3.10-dev \
    && rm -rf /var/lib/apt/lists/*
ARG LIBTORCH_URL=https://download.pytorch.org/libtorch/cu128/libtorch-shared-with-deps-2.9.1%2Bcu128.zip
RUN curl --fail --location --retry 3 "$LIBTORCH_URL" -o /tmp/libtorch.zip \
    && sha256sum /tmp/libtorch.zip > /opt/libtorch-download.sha256 \
    && unzip -q /tmp/libtorch.zip -d /opt && rm /tmp/libtorch.zip
ENV CUDA_HOME=/usr/local/cuda-12.8
ENV PATH=/usr/local/cuda-12.8/bin:${PATH}
ENV LD_LIBRARY_PATH=/opt/libtorch/lib:/usr/local/cuda-12.8/lib64:${LD_LIBRARY_PATH}
ENV CC=/usr/bin/gcc-11 CXX=/usr/bin/g++-11 CUDAHOSTCXX=/usr/bin/g++-11
ENV ROS_DOMAIN_ID=56
WORKDIR /workspace
ENTRYPOINT ["/bin/bash"]
CMD ["-lc", "sleep infinity"]
