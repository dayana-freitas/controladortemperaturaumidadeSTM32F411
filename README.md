# Controlador de Temperatura e Umidade com STM32F411

<a href="https://www.youtube.com/watch?v=th4r7HzCY8c&t=13s&ab_channel=DayanaFreitas">Apresentação do projeto no YouTube</a>

Projeto final desenvolvido para a disciplina de Sistemas Microcontrolados, ministrada pelo professor Delvanei Gomes Bandeira Jr, no ano de 2024/2025, na UTFPR.

<h3>Equipe:</h3>

- Cauã Felipe Gomes de Azevedo
- Dayana Resende de Freitas
- Santos Luis Mansogo Engono Nkara

Desenvolvemos o projeto a partir do controlador de temperatura e umidade N323-RHT da marca Novus, que possui três saídas de controle e configuração manual. Criamos um protótipo com funções de definição de temperatura e umidade mínimas e máximas conforme as necessidades do usuário. As saídas ativarão a irrigação, aquecimento e refrigeração da estufa conforme os parâmetros definidos. Além disso, um alerta luminoso será acionado quando as medições se aproximarem dos valores críticos.

A programação do controlador é baseada em interrupções, com dois timers (TIM4 e TIM5) funcionando a 1Hz. O TIM4 ativa uma rotina de interrupção para leitura dos potenciômetros e atualização das saídas do controlador. O TIM5 alterna a exibição de temperatura e umidade no display a cada interrupção. Um loop WHILE cadastra parâmetros quando o botão de função é pressionado, interrompendo os timers. O TIM3 é responsável pelo PWM do led de alerta. A máquina de estados controla a ativação das saídas, incluindo irrigação, aquecimento, resfriamento e alertas.
