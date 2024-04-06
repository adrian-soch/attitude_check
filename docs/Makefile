# Makefile
# TODO: Missing artifacts should be added to this file
.PHONY:	SRS

all:	SRS MG MIS PS VnVP VnVR Refl

SRS:
	cd SRS && make && cd ..

MG:
	cd Design/SoftArchitecture && make && cd ../..

MIS:
	cd Design/SoftDetailedDes && make && cd ../..

PS:
	cd ProblemStatementAndGoals && make && cd ..

VnVP:
	cd VnVPlan && make && cd ../..

VnVR:
	cd VnVReport && make && cd ../..

Refl:
	cd Reflection && make && cd ../..

clean:	cleanSRS cleanMG cleanMIS cleanPS cleanVnVP cleanVnVR cleanRefl

cleanPS:
	cd ProblemStatementAndGoals && make clean && cd ..

cleanSRS:
	cd SRS && make clean && cd ..

cleanMG:
	cd Design/SoftArchitecture && make clean && cd ..

cleanMIS:
	cd Design/SoftDetailedDes && make clean && cd ..

cleanVnVP:
	cd VnVPlan && make clean && cd ..

cleanVnVR:
	cd VnVReport && make clean && cd ..

cleanRefl:
        cd Reflection && make clean && cd ..
