
def add_file_downloads(confs : dict) -> list:
    '''
        @param confs kconfig vars, dict type
        @return list type, items is dict type
    '''
    if (not confs.get("PLATFORM_MAIXCAM", None)) or confs.get("CONFIG_COMPONENTS_COMPILE_FROM_SOURCE", None) or confs.get("CONFIG_OPENCV_COMPILE_FROM_SOURCE", None):
        # version = f"{confs['CONFIG_PYTHON_VERSION_MAJOR']}.{confs['CONFIG_PYTHON_VERSION_MINOR']}.{confs['CONFIG_PYTHON_VERSION_PATCH']}"
        version = "4.9.0"
        url = f"https://github.com/opencv/opencv/archive/{version}.zip"
        if version == "4.9.0":
            sha256sum = "9b5b64d50bf4a3ddeab430a9b13c5f9e023c9e67639ab50a74d0c298b5a61b74"
        else:
            raise Exception(f"version {version} not support")
        sites = ["https://github.com/sipeed/MaixCDK/releases/tag/v0.0.0"]
        filename = f"opencv-{version}.zip"
        path = f"opencv/opencv4"
        check_file = f'opencv-{version}'
        rename = {}

        files = [
            {
                'url': f'{url}',
                'urls': [],
                'sites': sites,
                'sha256sum': sha256sum,
                'filename': filename,
                'path': path,
                'check_files': [
                    check_file
                ],
                'rename': rename
            }
        ]
        if confs.get("PLATFORM_MAIXCAM", None):
            version = "0.1.2a"
            url = f"https://github.com/opencv/ade/archive/v0.1.2a.zip"
            if version == "0.1.2a":
                sha256sum = "4e32d16c56c2ecc0d8f4fc48cb9eb4af2be2048e7c71b430aa073b03bb8e1e48"
            else:
                raise Exception(f"version {version} not support")
            sites = ["https://github.com/opencv/ade"]
            filename = f"fa4b3e25167319cb0fa9432ef8281945-v0.1.2a.zip"
            path = f"opencv/cache/ade"
            check_file = f'opencv-{version}'
            rename = {}
            files = [
                {
                    'url': f'{url}',
                    'urls': [],
                    'sites': sites,
                    'sha256sum': sha256sum,
                    'filename': filename,
                    'path': path,
                    'check_files': [
                        check_file
                    ],
                    'rename': rename,
                    'extract': False
                }
            ]
        elif confs.get("PLATFORM_LINUX", None):
            files = [
                {
                    'url': f'https://github.com/opencv/ade/archive/v0.1.2b.zip',
                    'urls': [],
                    'sites': ["https://github.com/opencv/ade"],
                    'sha256sum': "a1d2a21e78a5a1b7bd41697a562669d5b5a8e2ab89e8a654c0a80a82aff28e19",
                    'filename': "4f93a0844dfc463c617d83b09011819a-v0.1.2b.zip",
                    'path': f"opencv/cache/ade",
                    'check_files': [
                        f'opencv-0.1.2b'
                    ],
                    'rename': {},
                    'extract': False
                },
                {
                    'url': f'https://raw.githubusercontent.com/opencv/opencv_3rdparty/1224f78da6684df04397ac0f40c961ed37f79ccb/ippicv/ippicv_2021.8_lnx_intel64_20230330_general.tgz',
                    'urls': [],
                    'sites': [],
                    'sha256sum': "7cfe0fb0e15ea8f3d2d971c19df2d14382469943d4efa85e48bf358930daa85d",
                    'filename': "43219bdc7e3805adcbe3a1e2f1f3ef3b-ippicv_2021.8_lnx_intel64_20230330_general.tgz",
                    'path': "opencv/cache/ippicv",
                    'check_files': [
                    ],
                    'rename': {},
                    'extract': False
                }
            ]
        else:
            raise Exception("No opencv config for this board, please edit to add opencv support for this board")

        return files

    if "musl" not in confs["CONFIG_TOOLCHAIN_PATH"]:
        return []
    # version = f"{confs['CONFIG_OMV_VERSION_MAJOR']}.{confs['CONFIG_OMV_VERSION_MINOR']}.{confs['CONFIG_OMV_VERSION_PATCH']}"
    version = "4.9.0"
    url = f"https://github.com/sipeed/MaixCDK/releases/download/v0.0.0/opencv4_lib_maixcam_musl_4.9.0.tar.xz"
    if version == "4.9.0":
        sha256sum = "c4553fe39212b271724d2bb9394e8944678f846f4e58a578da1db3013cbc3dcf"
    else:
        raise Exception(f"version {version} not support")
    sites = ["https://github.com/sipeed/MaixCDK/releases/tag/v0.0.0"]
    filename = f"opencv4_lib_maixcam_musl_4.9.0.tar.xz"
    path = f"opencv/opencv4"
    check_file = f'opencv4_lib_maixcam_musl_4.9.0'
    rename = {}

    return [
        {
            'url': f'{url}',
            'urls': [],
            'sites': sites,
            'sha256sum': sha256sum,
            'filename': filename,
            'path': path,
            'check_files': [
                check_file
            ],
            'rename': rename
        }
    ]

