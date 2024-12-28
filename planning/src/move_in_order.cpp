
#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MoveInOrder : public plansys2::ActionExecutorClient
{
public:
  MoveInOrder()
  : plansys2::ActionExecutorClient("move_in_order", 1s)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    // Simulazione di un movimento ordinato
    if (progress_ < 1.0) {
      progress_ += 0.1;  // Aumenta il progresso del 5% ad ogni chiamata
      send_feedback(progress_, "Moving in order");  // Invia feedback con il progresso
    } else {
      finish(true, 1.0, "Movement completed");  // Completa l'azione quando il progresso è 1.0

      progress_ = 0.0;  // Reset del progresso
      std::cout << std::endl;
    }

    // Stampa il progresso a schermo
    std::cout << "\r\e[K" << std::flush;
    std::cout << "Moving in order ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;  // Variabile per tenere traccia del progresso dell'azione
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);  // Inizializza il sistema ROS 2
  auto node = std::make_shared<MoveInOrder>();  // Crea un oggetto di tipo MoveInOrder

  node->set_parameter(rclcpp::Parameter("action_name", "move_in_order"));  // Imposta il nome dell'azione
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  // Esegue la transizione di configurazione

  rclcpp::spin(node->get_node_base_interface());  // Inizia l'esecuzione del nodo

  rclcpp::shutdown();  // Spegne il sistema ROS 2

  return 0;
}

