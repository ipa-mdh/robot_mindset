#!/usr/bin/env python3
from ament_index_python.packages import get_packages_with_prefixes


def get_spark_packages():
    available_packages = get_packages_with_prefixes()
    package_names = sorted(list(available_packages.keys()))
    
    spark_packages = []
    for package_name in package_names:
        if 'spark' in package_name:
            print(f"Found package: {package_name}")
            spark_packages.append(package_name)
    
    return spark_packages