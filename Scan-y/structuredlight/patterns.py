class Patterns:
    def gray(self, dsize):
        width, height = dsize
        patternCnt = int(math.log2(width))+1
        #<<Горното>> = width/pow(2,x) - през колко трябва да се сменят 0/1;
        #(y/(width/pow(2,x)))%2 - ако y/<<Горното>> е четно, то 0, иначе 1
        print(np.fromfunction(lambda x,y: (y/(width/pow(2,x)))%2, (patternCnt,width), dtype=int).astype(np.uint8))
