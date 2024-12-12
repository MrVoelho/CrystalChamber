# Crystal Chamber
Uma câmara DIY (Faça-você-mesmo) para crescimento de cristais salinos pelo método de evaporação com controle de temperatura e umidade relativa.

A câmara é construída com componentes facilmente acessíveis, utiliza sílica gel como agente dissecante, uma pastilha termoelétrica para resfriamento e um Arduino para monitorar e regular as condições ambientais. Também inclui um display para controle e acompanhamento com quatro modos operacionais (Temperatura e umidade relativa, só temperatura, só umidade e sem controle). Além disso, um circuido de pesagem com célula de carga pode ser incorporado para medir diretamente a variação de massa da solução ao longo do tempo.

> This project is also avaiable in english, check the "English" folder
## Construção
Uma lista dos componentes necessários e esquemáticos para construir sua própria Crystal Chamber estão no arquivo de Documentação, juntamente à descrição do algorítmo de controle no sketch Arduino (totalmente comentado) e algumas imagens para orientação.

Este projeto foi pensado em etapas, da observação para o controle de umidade e, finalmente, o controle de temperatura. Então, caso você não tenha acesso a toda a lista de materiais, ou só queira começar devagar, sem problema! Consulte a seção `Conexões` na Documentação e ajuste o sketch de acordo com o que estiver disponível.

Conhecimento básico de eletrônica é necessário para construir este projeto, incluindo solda e manuseio de componentes.
## Operação
Uma vez que o crescimento de cristais é um processo lento, o algorítmo foi desenvolvido com foco na autonomia, minimizando a necessidade de intervenções. Também existem algumas verificações de segurança para garantir a qualidade dos cristais (ex. nível mínimo de umidade) e condições operacionais (ex. caso um sensor falhe). A descrição dos modos operacionais também estão na Documentação.

Uma vez que a câmara esteja pronta, ajuste as faixas de temperatura e umidade no sketch, faça upload para seu arduino, carregue um pouco de sílica gel no pote dessecante e cultive seus cristais!

## Um pouco de teoria
Para mais informação sobre o crescimento de cristais de soluções salinas e a física por trás, veja o vídeo [Home-based science of crystal growing](https://youtu.be/u3r0Pdgs1Jw). Por enquanto disponível apenas em inglês.

## ChangeLog
v1.1.G (december/2024) 
- Primeira versão publicada.

## License 
GPL v3.0 License
