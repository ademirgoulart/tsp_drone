#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <ctime>
#include <math.h>
#include <iomanip>
using namespace std;

// avalia pelo melhor caminho de um ponto ao proximo considerando
// a combinação de menor caminho e prioridade de acesso ao ponto
// adaptado de UAV Path Planning for Emergency Management in IoT de
// Sherin Abdelhamid 

int main(int argc, char *argv[])
{
	FILE *arqEntrada;
	// Opção usando distancia e prioridade em cada nó
	// velocidade do drone modo t 18km/h ou 0,3km/m
	// duração da bateria 0,5 h
	// Dmax = Distancia maxima D=18* 0,5 = 9000 metros 
	// Tempo de parada em cada nó 2 minutos -> 
	// Deq = Distancia equivalente nos 2 minutos Deq=0,3*2 = 600 metros 
	int n=11 ;  //  numero de nós 
	int Deq=600, Dmax=9000;
	int i, j, ic=0 , iv=0, im, totdis=0, tp=0;
	float maior=0.0;
	int totdv=0;
	int xs[n], ys[n], ps[n], maxw=0;
	float maxd=0;
	int matd [n] [n];
	float matdn [n][n], matsi[n][n];
	char nome [256];
	int visitado [n]; // vetor se visitado = 1
	int caminho [n]; //vetor com caminho selecionado
	int min [n];  //  vetor com distancia minima em cada linha matriz distancia
	int max [n];  //  vetor com distancia maxima em cada linha matriz distancia
	// inicializa visitado e caminho
	for (i=0; i<n;i++){
		visitado[i]=0;
		caminho[i]=0;			
	}

	// leitura valores do arquivo entrada
	cout << "Informar nome arquivo entrada : ";
	cin >> nome;
	arqEntrada = fopen (nome, "r");

	
	//  parametros de entrada:  coordenada X, coordenada Y, Prioridade no ponto
	for (i=0; i<n;i++){
		fscanf (arqEntrada, "%d %d %d\n", &xs[i], &ys[i], &ps[i]);			
	}
	fclose(arqEntrada);
	// Lista os dados de entrada
	for (i=0; i<n;i++){
		cout << "X=" << xs[i]  << "\t Y=" << ys[i] <<  "\t Prioridade= " << ps[i] <<"\n";
	}
		
	// calcular distancias dos pontos
	
	for(i=0;i<n;i++){
		for (j=0;j<n;j++){
			matd[i][j]=  sqrt (pow ((xs[j]-xs[i]),2)+pow ((ys[j]-ys[i]),2));
			if (matd[i][j]> maxd) maxd=matd[i][j];
		}	
	}
	
	//  verifica maxw  maximo valor de prioridade
	
	for (i=0; i<n;i++){
		if (ps[i] > maxw) maxw = ps [i];
		}
	cout << "\n maxima prioridade = " << maxw  << "\n"; 
	
		
	//  imprimir matriz com distancias 
	cout <<"\n Matriz com distancias";
	cout << "\n\t";
	for (j=0;j<n;j++){
		cout << j << "\t";
	}
	cout << "min\tmax\n";	
	
	for(i=0;i<n;i++){
		cout << i << ":\t";
		min[i]=matd[i][0];
		max[i]=matd[i][0];
		for (j=0;j<n;j++){
			if (matd[i][j] < min[i] ) min[i]=matd[i][j];
			if (matd[i][j] > max[i]) max[i]=matd[i][j];
			cout << matd[i][j] << "\t";
		}
		cout << min[i]  << "\t"  << max[i];
		cout << "\n";
	}
		
	//  calcula matriz distancias normalizada X  prioridade maxima
	//  matd = distancia normalizada * prioridade maxima
	//  normaliza a distancia para ficar no mesmo range da prioridade
	//  dn_x = (d_x - d_minima)/(d_maxima-d_minima)
	//  no problema atual d_minima é sempre zero
		for(i=0;i<n;i++){
			for (j=0;j<n;j++){
				matdn[i][j]=  (matd[i][j] / (float) max[i]) * maxw ;
			}	
		}
	
	//  imprime matriz com distancias  normalizada  
	cout <<"\n Matriz com distancias normalizadas x maxima prioridade";
	cout << "\n\t";
	for (j=0;j<n;j++){
		cout << j << "\t";
	}
	cout << "\n";	
	
	for(i=0;i<n;i++){
		cout << i << ":\t";
		for (j=0;j<n;j++){
			std::cout << std::fixed << std::setprecision(2) << matdn[i][j];
			cout  << "\t";
		}
		cout << "\n";
	}
	
	//   calcula matsi  com wi/matdn 
	for(i=0;i<n;i++){
			for (j=0;j<n;j++){
				if(matdn[i][j] !=0) matsi[i][j]= ps[j] / matdn[i][j];
				else  matsi[i][j]= 0;
			}	
	}
	
	//  imprimir matriz com indices Si   
	cout <<"\n Matriz com indices Si prioridade do ponto / dist normalizada";
	cout << "\n\t";
	for (j=0;j<n;j++){
		cout << j << "\t";
	}
	cout << "\n";	
	
	for(i=0;i<n;i++){
		cout << i << ":\t";
		for (j=0;j<n;j++){
			std::cout << std::fixed << std::setprecision(2) << matsi[i][j];
			cout  << "\t";
		}
		cout << "\n";
	}	
	
	// rotina principal procura menor valor de distancia / prioridade
	// marca este ponto e ve o proximo.
	// na matriz matsi o indice de maior valor tem a melhor escolha combinando
	// menor distancia e prioridade do ponto
	ic=1;
	i=0;
	visitado[0]=1;
	caminho[0]=0;
	cout << "\nOrigem \tDestino    Distancia \tDistancia \tDistancia \tDistancia \tStatus";
	cout << "\n \t\t\t\tAcumulada \tRetorno \tTotal";
	while (ic < n){
		maior=0.0;
		for (j=0; j<n;j++){
			if (matsi[i][j] > maior && visitado[j] != 1){
				maior = matsi[i][j];
				im = j;
			}
		}
		totdis=totdis+matd[i][im] + Deq;
		totdv = totdis + matd[0][im];
		caminho[ic]=im;
		visitado[im]=1;
		ic++;
		// tdvolta total deste ponto + volta ao ponto 0
		cout << "\n   " << i  << "\t   " << im << "\t\t " << 
		matd[i][im]<< "\t " << totdis << "\t\t " << totdv 
		<<"\t\t  " << totdis+totdv ;
		if ((totdis+totdv) < Dmax ) {
			cout << "\t\t <- ok";
			tp++;
		}		
		i=im;		
	}
	//  imprime caminho
	cout << "\n\n Caminho = ";
	for(i=0;i<=tp;i++){
		cout << " " << caminho[i];
	}
	return 0;	
}
