from setuptools import setup, find_packages
import os

def read_requirements():
    requirements_file = 'requirements.txt'
    if os.path.exists(requirements_file):
        with open(requirements_file, 'r') as f:
            return [line.strip() for line in f if line.strip() and not line.startswith('#')]
    return []

def read_readme():
    readme_file = 'README.md'
    if os.path.exists(readme_file):
        with open(readme_file, 'r', encoding='utf-8') as f:
            return f.read()
    return ""

setup(
    name='adore-model-checker',
    version='0.1.0',
    description="Dynamic ROS Model Checker for Vehicle Safety Monitoring and Verification",
    long_description=read_readme(),
    long_description_content_type="text/markdown",
    author="Your Name",
    author_email="your.email@example.com",
    url="https://github.com/yourusername/adore-model-checker",
    
    packages=find_packages(include=['adore_model_checker', 'adore_model_checker.*']),
    py_modules=['adore_model_checker_cli', 'adore_model_checker_api_app'],

    include_package_data=True,
    package_data={
        'adore_model_checker': [
            'config/*.yaml', 
            'config/*.yml',
            '*.yaml', 
            '*.yml'
        ],
        '': ['config/*.yaml', 'config/*.yml'],
    },
    
    install_requires=read_requirements(),
    python_requires='>=3.8',

    entry_points={
        'console_scripts': [
            'adore-model-checker = adore_model_checker_cli:main',
            'adore-model-checker-api = adore_model_checker_api_app:main',
        ],
    },

    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development :: Testing',
        'Topic :: System :: Monitoring',
    ],
    
    keywords='ros2 model-checking safety verification autonomous-vehicles temporal-logic',

    project_urls={
        'Bug Reports': 'https://github.com/yourusername/adore-model-checker/issues',
        'Source': 'https://github.com/yourusername/adore-model-checker',
        'Documentation': 'https://github.com/yourusername/adore-model-checker/wiki',
    },

    zip_safe=False,

    extras_require={
        'dev': [
            'pytest>=6.0',
            'pytest-cov',
            'black',
            'flake8',
            'mypy',
        ],
        'api': [
            'flask>=2.0',
            'werkzeug>=2.0',
        ],
        'plotting': [
            'matplotlib>=3.0',
            'seaborn>=0.11',
        ],
        'all': [
            'pytest>=6.0',
            'pytest-cov',
            'black',
            'flake8',
            'mypy',
            'flask>=2.0',
            'werkzeug>=2.0',
            'matplotlib>=3.0',
            'seaborn>=0.11',
        ]
    },
)
