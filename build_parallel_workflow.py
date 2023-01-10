import os

WORKFLOW_TEMPLATE = """
# This is a faster workflow that parallelizes the jobs in a matrix so
# we can get faster results than waiting for the standard build_all_frc_projects
# powershell script

name: Build all FRC Projects

on:
  push:
    branches: [ "master" ]
  pull_request:
    branches: [ "master" ]

permissions:
  contents: read

jobs:
  build:

    strategy:
      fail-fast: false
      matrix:
        include:{projects_as_matrix}

    # The type of runner that the job will run on
    runs-on: ubuntu-latest

    # This grabs the WPILib docker container
    container: wpilib/roborio-cross-ubuntu:2023-22.04

    steps:
    - uses: actions/checkout@v3

    # Grant execute permission for gradlew
    - name: Grant execute permission for gradlew
      run: cd "${{{{ matrix.directory }}}}" && chmod +x gradlew

    # Runs a single command using the runners shell
    - name: Compile and run tests on robot code for project ${{{{ matrix.project-name }}}}
      run: cd "${{{{ matrix.directory }}}}" && ./gradlew build
"""

PROJECT_MATRIX_TEMPLATE = """
          - project-name: '{project_name}'
            directory: '{project_dir}'"""

PROJECTS_TO_SEARCH = ["C++ General", "C++ Talon FX (Falcon 500)", "Java General", "Java Talon FX (Falcon 500)"]


project_matrix = []
for project_dir in PROJECTS_TO_SEARCH:
    # Find every project in here and build up an array of strings to generate the workflow file
    for project in os.listdir(project_dir):
        project_matrix.append(PROJECT_MATRIX_TEMPLATE.format(project_name=project, project_dir=f"{project_dir}/{project}"))

with open(".github/workflows/build-all-parallel.yml", "w", encoding="utf-8") as workflow_file:
    workflow_file.write(WORKFLOW_TEMPLATE.format(projects_as_matrix="".join(project_matrix)))
