# Phoenix Example

## Purpose
This is a self-contained example that demonstrates an ability of one of CTRE's products.
It is tested to ensure the ability works as expected, and provided so that users have a starting point on how to use it.

## Managing Phoenix Version
These examples have a Phoenix Version that they come with. This version is the version that was tested on the example.
If the user wants to update the version, they can do so by following the [Phoenix Documentation](https://phoenix-documentation.readthedocs.io/en/latest/ch05a_CppJava.html#frc-c-java-add-phoenix).
Unless specified, Phoenix Updates **are** backwards compatible, so updating your Phoenix version is harmless.

## How to manage WPILib Version
Each example has a WPILib version associated with it. This version is the earliest legal version of WPILib available (oftentimes the kickoff version). This is to ensure users with old WPILib versions are able to use our examples.
If you would like to update the WPILib version, press the WPILib logo in the upper right, and select "Check for WPILib Updates".

## WPILib-Old-Commands vs WPILib-New-Commands
When a project is imported from 2019, a WPILib-Old-Commands json is automatically added to the project. This is because it assumes the project uses a command-based framework. 
None of CTR's examples use a command-based framework so all of these jsons have been removed.

If you would like more information about the Old-Command to New-Command changes, look at the [WPILIB Documentation](https://docs.wpilib.org/en/latest/docs/software/commandbased/command-based-changes.html)