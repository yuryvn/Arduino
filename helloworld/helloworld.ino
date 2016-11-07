
/*const int LedPin[14] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13};
*/

class AlphabetT{
  public:
    AlphabetT();
    ~AlphabetT();
    
  int *GetLetter(char Input);
  int GetLetterSize()const;

    
  private:
  int Size;
  const int LetterASize, LetterBSize,LetterCSize,LetterDSize,LetterESize,LetterFSize,LetterGSize,LetterHSize,LetterISize,LetterJSize,LetterKSize,LetterLSize,LetterMSize,LetterNSize,LetterOSize,LetterPSize;
  const int LetterQSize, LetterRSize,LetterSSize,LetterTSize,LetterUSize,LetterVSize,LetterWSize,LetterXSize,LetterYSize,LetterZSize;
  
  int *LetterA,*LetterB,*LetterC,*LetterD,*LetterE,*LetterF,*LetterG,*LetterH,*LetterI,*LetterJ,*LetterK,*LetterL,*LetterM,*LetterN,*LetterO,*LetterP,*LetterQ,*LetterR,*LetterS,*LetterT,*LetterU,*LetterV,*LetterW,*LetterX,*LetterY,*LetterZ;
  void FillLetter(int *LetterPTR,int size,int a1=0,int a2=0,int a3=0, int a4=0, int a5=0, int a6=0, int a7=0, int a8=0, int a9=0);
};



AlphabetT::AlphabetT()
  :LetterASize(6), LetterBSize(9),LetterCSize(7),LetterDSize(6),LetterESize(8),LetterFSize(6),LetterGSize(9),LetterHSize(7),LetterISize(7),LetterJSize(6),LetterKSize(6),LetterLSize(5),LetterMSize(7),LetterNSize(7),LetterOSize(8),LetterPSize(7),LetterQSize(6), LetterRSize(8),LetterSSize(9),LetterTSize(5),LetterUSize(7),LetterVSize(5),LetterWSize(7),LetterXSize(5),LetterYSize(4),LetterZSize(7)
  {
  LetterA=(new int[LetterASize]);
  FillLetter(LetterA,LetterASize,3,5,8,7,10,6);

  LetterB=new int[LetterBSize];
  FillLetter(LetterB,LetterBSize,2,5,8,3,4,7,6,10,9);
  
  LetterD=new int[LetterDSize];
  FillLetter(LetterD,LetterDSize,2,5,8,9,7,3);
  

  LetterE=new int[LetterESize];
  FillLetter(LetterE,LetterESize,2,5,8,9,10,6,3,4);
  
  LetterF=new int[LetterFSize];
  FillLetter(LetterF,LetterFSize,2,5,8,3,4,6);
  
  LetterG=new int[LetterGSize];
  FillLetter(LetterG,LetterGSize,4,3,2,5,8,9,10,7,6);
  
  LetterH=new int[LetterHSize];
  FillLetter(LetterH,LetterHSize,2,5,8,6,7,4,10);
  
  LetterI=new int[LetterISize];
  FillLetter(LetterI,LetterISize,3,6,9,2,4,8,10);
  
  LetterJ=new int[LetterJSize];
  FillLetter(LetterJ,LetterJSize,3,6,9,8,2,4);
  
  LetterK=new int[LetterKSize];
  FillLetter(LetterK,LetterKSize,2,5,8,4,6,10);
  
  LetterL=new int[LetterLSize];
  FillLetter(LetterL,LetterLSize,2,5,8,9,10);
  
  LetterM=new int[LetterMSize];
  FillLetter(LetterM,LetterMSize,8,5,2,6,4,7,10);
  
  LetterN=new int[LetterNSize];
  FillLetter(LetterN,LetterNSize,8,5,2,6,10,7,4);
  
  LetterO=new int[LetterOSize];
  FillLetter(LetterO,LetterOSize,2,5,8,9,10,7,4,3);
  
  LetterP=new int[LetterPSize];
  FillLetter(LetterP,LetterPSize,2,5,8,6,7,4,3);
  
  LetterQ=new int[LetterQSize];
  FillLetter(LetterQ,LetterQSize,2,5,6,3,7,10);

  LetterR=new int[LetterRSize];
  FillLetter(LetterR,LetterRSize,2,5,8,3,4,7,6,10);
  
  LetterS=new int[LetterSSize];
  FillLetter(LetterS,LetterSSize,4,3,2,5,6,7,10,9,8);
  
  LetterT=new int[LetterTSize];
  FillLetter(LetterT,LetterTSize,3,6,9,2,4);
  
  LetterU=new int[LetterUSize];
  FillLetter(LetterU,LetterUSize,2,5,8,9,10,7,4);
  
  LetterV=new int[LetterVSize];
  FillLetter(LetterV,LetterVSize,2,5,9,7,4);

  LetterW=new int[LetterWSize];
  FillLetter(LetterW,LetterWSize,2,5,8,6,10,7,4);
  
  LetterX=new int[LetterXSize];
  FillLetter(LetterX,LetterXSize,2,6,10,4,8);
  
  LetterY=new int[LetterYSize];
  FillLetter(LetterY,LetterYSize,2,6,4,9);
  
  LetterZ=new int[LetterZSize];
  FillLetter(LetterZ,LetterZSize,2,3,4,6,8,9,10);
 
  }


AlphabetT::~AlphabetT(){}

void AlphabetT::FillLetter(int *LetterPTR,int size,int a1,int a2,int a3, int a4, int a5, int a6, int a7, int a8, int a9){
  int letter[9]={a1,a2,a3,a4,a5,a6,a7,a8,a9};
  for (int i=0;i<size;*(LetterPTR+i)=letter[i],i++);
}

int AlphabetT::GetLetterSize()const{
    return Size;
    }

int *AlphabetT::GetLetter(char Input){
    switch (Input) {
      case 'A':case 'a':
        Size=LetterASize;
        return LetterA;
        break;
      case 'B': case 'b':
        Size=LetterBSize;
        return LetterB;
        break;
      case 'C': case 'c':
        Size=LetterCSize;
        return LetterC;
        break;
      case 'D': case 'd':
        Size=LetterDSize;
        return LetterD;
        break;
      case 'E': case 'e':
        Size=LetterESize;
        return LetterE;
        break;
      case 'F': case 'f':
        Size=LetterFSize;
        return LetterF;
        break;
      case 'G': case 'g':
        Size=LetterGSize;
        return LetterG;
        break;
      case 'H': case 'h':
        Size=LetterHSize;
        return LetterH;
        break;
      case 'I': case 'i':
        Size=LetterISize;
        return LetterI;
        break;
      case 'J': case 'j':
        Size=LetterJSize;
        return LetterJ;
        break;
      case 'K': case 'k':
        Size=LetterKSize;
        return LetterK;
        break;
      case 'L': case 'l':
        Size=LetterLSize;
        return LetterL;
        break;
      case 'M': case 'm':
        Size=LetterMSize;
        return LetterM;
        break;
      case 'N': case 'n':
        Size=LetterNSize;
        return LetterN;
        break;
      case 'O': case 'o':
        Size=LetterOSize;
        return LetterO;
        break;
      case 'P': case 'p':
        Size=LetterPSize;
        return LetterP;
        break;
      case 'Q': case 'q':
        Size=LetterQSize;
        return LetterQ;
        break;
      case 'R': case 'r':
        Size=LetterRSize;
        return LetterR;
        break;
      case 'S': case 's':
        Size=LetterSSize;
        return LetterS;
        break;
      case 'T': case 't':
        Size=LetterTSize;
        return LetterT;
        break;
      case 'U': case 'u':
        Size=LetterUSize;
        return LetterU;
        break;
      case 'V': case 'v':
        Size=LetterVSize;
        return LetterV;
        break;
      case 'W': case 'w':
        Size=LetterWSize;
        return LetterW;
        break;
      case 'X': case 'x':
        Size=LetterXSize;
        return LetterX;
        break;
      case 'Y': case 'y':
        Size=LetterYSize;
        return LetterY;
        break;
      case 'Z': case 'z':
        Size=LetterZSize;
        return LetterZ;
        break;

        
      default:
        Size=LetterCSize;
        return LetterC;
        break;
    }
  }

const int Delay=80;


/*subriutines for showing a letter*/

void DisplayLetter(const int *LetterArray,const int size){
  for (int i=0;i<size;i++){
    digitalWrite(*(LetterArray+i), HIGH);
    delay(Delay);
    }
  delay(1000);
  for (int i=0;i<size;i++){
    digitalWrite(*(LetterArray+i), LOW);
 //   delay(Delay);
    }
   delay(100);
}

void setup()
{

  /* add setup code here */
for(int i=2;i<14;i++){
	pinMode(i, OUTPUT);
  digitalWrite(i,LOW);
}


}

AlphabetT Alphabet;
int *Letter;
int LetterSize;
const int Length=6;
char SV[Length]={'Z','H','e','n','y','a'};

void loop()
{
  /* add main program code here */
  for (int i=0;i<Length;i++){
    Letter=Alphabet.GetLetter(SV[i]);
    LetterSize=Alphabet.GetLetterSize();
    DisplayLetter(Letter,LetterSize);
  }

  delay(2000);
for (int k=0;k<3;k++){
  for(int i=2;i<14;i++){
  pinMode(i, OUTPUT);
  digitalWrite(i,HIGH);
}
delay(50);
for(int i=2;i<14;i++){
  pinMode(i, OUTPUT);
  digitalWrite(i,LOW);
}
delay(50);}
delay (500);

	/*for (int i=2;i<11;digitalWrite(i++, HIGH),delay(200));
	delay(2000);
	for (int i=2;i<11;digitalWrite(i++, LOW),delay(200));
	delay(2000);
  */
}
