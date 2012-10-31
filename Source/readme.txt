==========================================================================
                  Template Project for 96675r Series                   
==========================================================================

                        Warranty and Disclaimer 

 
The use of the deliverables (e.g. software, application examples, target 
boards, evaluation boards, starter kits, schematics, engineering samples 
of IC’s etc.) is subject to the conditions of Fujitsu Semiconductor 
Europe GmbH (“FSEU”) as set out in (i) the terms of the License 
Agreement and/or the Sale and Purchase Agreement under which agreements 
the Product has been delivered, (ii) the technical descriptions and 
(iii) all accompanying written materials. Please note that the 
deliverables are intended for and must only be used for reference in an 
evaluation laboratory environment. The software deliverables are 
provided on an as-is basis without charge and are subject to 
alterations. It is the user’s obligation to fully test the software in 
its environment and to ensure proper functionality, qualification and 
compliance with component specifications. Regarding hardware 
deliverables, FSEU warrants that they will be free from defects in 
material and workmanship under use and service as specified in the 
accompanying written materials for a duration of 1 year from the date of 
receipt by the customer. Should a hardware deliverable turn out to be 
defect, FSEU’s entire liability and the customer’s exclusive remedy 
shall be, at FSEU´s sole discretion, either return of the purchase price 
and the license fee, or replacement of the hardware deliverable or parts 
thereof, if the deliverable is returned to FSEU in original packing and 
without further defects resulting from the customer’s use or the 
transport. However, this warranty is excluded if the defect has resulted 
from an accident not attributable to FSEU, or abuse or misapplication 
attributable to the customer or any other third party not relating to 
FSEU or to unauthorised decompiling and/or reverse engineering and/or 
disassembling. FSEU does not warrant that the deliverables do not 
infringe any third party intellectual property right (IPR). In the event 
that the deliverables infringe a third party IPR it is the sole 
responsibility of the customer to obtain necessary licenses to continue 
the usage of the deliverable. In the event the software deliverables 
include the use of open source components, the provisions of the 
governing open source license agreement shall apply with respect to such 
software deliverables. To the maximum extent permitted by applicable law 
FSEU disclaims all other warranties, whether express or implied, in 
particular, but not limited to, warranties of merchantability and 
fitness for a particular purpose for which the deliverables are not 
designated. To the maximum extent permitted by applicable law, FSEU’s 
liability is restricted to intention and gross negligence. FSEU is not 
liable for consequential damages. Should one of the above stipulations 
be or become invalid and/or unenforceable, the remaining stipulations 
shall stay in full effect. The contents of this document are subject to 
change without a prior notice, thus contact FSEU about the latest one. 

(V1.2) 


==========================================================================
History
Date        Ver     Author  Softune     Description
2011-02-03  1.0     CEy     V30L35      original version
==========================================================================

This is a project template for the MB96F675R devices. It includes some basic 
settings for e.g. Linker, C-Compiler which must be checked and modified in 
detail, corresponding to the user application.

Clock settings:
---------------
Crystal:  4 MHz
CLKB:    32 MHz
CLKP1:   32 MHz
CLKP2:   16 MHz