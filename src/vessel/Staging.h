#pragma once
#include <algorithm>
#include <map>
#include <memory>
#include <vector>
#include "Part.h"

namespace DeepSpace {

    class StagingSystem {
    public:
        void RebuildStages(const std::vector<std::shared_ptr<Part>>& parts) {
            m_Stages.clear();
            int maxStage = -1;
            for (const auto& part : parts) {
                if (part->GetStage() >= 0) {
                    m_Stages[part->GetStage()].push_back(part);
                    maxStage = std::max(maxStage, part->GetStage());
                }
            }
            m_CurrentStage = maxStage;
        }
        
        bool ActivateNextStage() {
            if (m_CurrentStage < 0 || m_Stages.empty()) return false;

            auto currentIt = m_Stages.find(m_CurrentStage);
            if (currentIt == m_Stages.end()) {
                --m_CurrentStage;
                return true;
            }

            for (auto& part : currentIt->second) {
                if (auto engine = std::dynamic_pointer_cast<EnginePart>(part)) {
                    engine->SetActive(true);
                    engine->SetThrottle(1.0);
                }
                if (auto decoupler = std::dynamic_pointer_cast<DecouplerPart>(part)) {
                    decoupler->Activate();
                }
            }

            bool stageHasDecoupler = false;
            for (const auto& part : currentIt->second) {
                if (std::dynamic_pointer_cast<DecouplerPart>(part)) {
                    stageHasDecoupler = true;
                    break;
                }
            }

            if (stageHasDecoupler) {
                for (const auto& pair : m_Stages) {
                    if (pair.first > m_CurrentStage) {
                        for (auto& p : pair.second) {
                            if (!p->IsPersistent()) {
                                p->SetDecoupled(true);
                            }
                        }
                    }
                }
            }

            --m_CurrentStage;
            return true;
        }

        int GetCurrentStage() const { return m_CurrentStage; }

    private:
        std::map<int, std::vector<std::shared_ptr<Part>>> m_Stages;
        int m_CurrentStage = -1;
    };
}
